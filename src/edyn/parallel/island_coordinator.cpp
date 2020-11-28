#include "edyn/parallel/island_coordinator.hpp"
#include "edyn/parallel/island_worker.hpp"
#include <entt/entt.hpp>

namespace edyn {

template<typename T>
bool vector_contains(const std::vector<T> &vec, const T &value) {
    return std::find(vec.begin(), vec.end(), value) != vec.end();
}

island_coordinator::island_coordinator(entt::registry &registry)
    : m_registry(&registry)
{
    registry.on_construct<island_node>().connect<&island_coordinator::on_construct_island_node>(*this);
    registry.on_destroy<island_node>().connect<&island_coordinator::on_destroy_island_node>(*this);
    registry.on_destroy<island_container>().connect<&island_coordinator::on_destroy_island_container>(*this);

    registry.on_construct<constraint>().connect<&island_coordinator::on_construct_constraint>(*this);
    registry.on_destroy<constraint>().connect<&island_coordinator::on_destroy_constraint>(*this);
}

island_coordinator::~island_coordinator() {
    for (auto &pair : m_island_info_map) {
        auto &info = pair.second;
        info->m_worker->terminate();
    }

    for (auto &pair : m_island_info_map) {
        auto &info = pair.second;
        info->m_worker->join();
    }
}

void island_coordinator::on_construct_island_node(entt::registry &registry, entt::entity entity) {
    if (m_importing_snapshot) return;

    registry.emplace<island_container>(entity);
    m_new_island_nodes.push_back(entity);
}

void island_coordinator::on_destroy_island_node(entt::registry &registry, entt::entity entity) {
    if (m_importing_snapshot) return;

    auto &node = registry.get<island_node>(entity);

    // Remove from connected nodes.
    for (auto other : node.entities) {
        auto &other_node = registry.get<island_node>(other);
        other_node.entities.erase(
            std::remove(
                other_node.entities.begin(),
                other_node.entities.end(), entity), 
            other_node.entities.end());
        registry.get_or_emplace<island_node_dirty>(other).indexes.insert(entt::type_index<island_node>::value());
    }
}

void island_coordinator::on_destroy_island_container(entt::registry &registry, entt::entity entity) {
    if (m_importing_snapshot) return;

    auto &container = registry.get<island_container>(entity);

    // Remove from islands.
    for (auto island_entity : container.entities) {
        auto &info = m_island_info_map.at(island_entity);
        info->m_snapshot_builder.destroyed(entity);
    }
}

void island_coordinator::init_new_island_nodes() {
    std::unordered_set<entt::entity> procedural_node_entities;

    for (auto entity : m_new_island_nodes) {
        if (m_registry->has<procedural_tag>(entity)) {
            procedural_node_entities.insert(entity);
        } else {
            init_new_non_procedural_island_node(entity);
        }
    }

    m_new_island_nodes.clear();

    while (!procedural_node_entities.empty()) {
        auto node_entity = *procedural_node_entities.begin();

        // Find connected components to build new islands for procedural entities.
        std::unordered_set<entt::entity> island_entities;
        std::unordered_set<entt::entity> connected;
        std::vector<entt::entity> to_visit;
        to_visit.push_back(node_entity);

        while (!to_visit.empty()) {
            auto curr_entity = to_visit.back();
            to_visit.pop_back();

            // Add to connected component.
            connected.insert(curr_entity);

            // Remove from main set.
            procedural_node_entities.erase(curr_entity);

            // Add related entities to be visited next.
            auto &curr_node = m_registry->get<island_node>(curr_entity);

            for (auto other_entity : curr_node.entities) {
                auto already_visited = connected.count(other_entity) > 0;
                if (already_visited) continue;

                if (m_registry->has<procedural_tag>(other_entity)) {
                    to_visit.push_back(other_entity);

                    // Collect islands involved in this connected component.
                    auto &other_container = m_registry->get<island_container>(other_entity);
                    if (!other_container.entities.empty()) {
                        // Procedural entity must be in only one island.
                        EDYN_ASSERT(other_container.entities.size() == 1);
                        auto island_entity = other_container.entities.front();
                        island_entities.insert(island_entity);
                    }
                } else {
                    // Non-procedural nodes must not be visited because they do
                    // not provide a path to connect other nodes through itself.
                    connected.insert(other_entity);
                }
            }
        }

        entt::entity island_entity;

        if (island_entities.empty()) {
            const auto timestamp = (double)performance_counter() / (double)performance_frequency();
            island_entity = create_island(timestamp);
        } else if (island_entities.size() > 1) {
            island_entity = merge_islands(island_entities, connected);
        } else {
            island_entity = *island_entities.begin();
        }

        auto &info = m_island_info_map.at(island_entity);

        for (auto entity : connected) {
            // Assign island to containers.
            auto &container = m_registry->get<island_container>(entity);
            if (!vector_contains(container.entities, island_entity)) {
                container.entities.push_back(island_entity);
            }

            // Add new entities to the snapshot builder.
            info->m_snapshot_builder.created(entity);
            info->m_snapshot_builder.maybe_updated(entity, *m_registry, all_components{});
        }
    }
}

void island_coordinator::init_new_non_procedural_island_node(entt::entity node_entity) {
    auto &node = m_registry->get<island_node>(node_entity);
    auto &container = m_registry->get<island_container>(node_entity);

    EDYN_ASSERT(!(m_registry->has<procedural_tag>(node_entity)));

    // Add new non-procedural entity to islands of related procedural entities.
    for (auto other : node.entities) {
        if (!m_registry->has<procedural_tag>(other)) continue;

        auto &other_container = m_registry->get<island_container>(other);
        if (other_container.entities.empty()) continue;

        auto island_entity = other_container.entities.front();

        if (!vector_contains(container.entities, island_entity)) {
            container.entities.push_back(island_entity);
        }
        
        auto &info = m_island_info_map.at(island_entity);
        info->m_snapshot_builder.created(node_entity);
        info->m_snapshot_builder.maybe_updated(node_entity, *m_registry, all_components{});
    }
}

entt::entity island_coordinator::create_island(double timestamp) {
    auto entity = m_registry->create();
    m_registry->emplace<island>(entity);
    auto &isle_time = m_registry->emplace<island_timestamp>(entity);
    isle_time.value = timestamp;

    auto [main_queue_input, main_queue_output] = make_message_queue_input_output();
    auto [isle_queue_input, isle_queue_output] = make_message_queue_input_output();

    // The `island_worker` is dynamically allocated and kept alive while
    // the associated island lives. The job that's created for it calls its
    // `update` function which reschedules itself to be run over and over again.
    // After the `finish` function is called on it (when the island is destroyed),
    // it will be deallocated on the next run.
    auto *worker = new island_worker(entity, m_fixed_dt, message_queue_in_out(main_queue_input, isle_queue_output));
    auto info = std::make_unique<island_info>(entity, worker, message_queue_in_out(isle_queue_input, main_queue_output));

    // Send over a snapshot containing this island entity to the island worker
    // before it even starts.
    auto builder = registry_snapshot_builder(info->m_entity_map);
    builder.created(entity);
    builder.maybe_updated(entity, *m_registry, all_components{});
    info->m_message_queue.send<registry_snapshot>(builder.get_snapshot());

    if (m_paused) {
        info->m_message_queue.send<msg::set_paused>(msg::set_paused{true});
    }

    // Register to receive snapshots.
    info->registry_snapshot_sink().connect<&island_coordinator::on_registry_snapshot>(*this);

    m_island_info_map.emplace(entity, std::move(info));

    auto j = job();
    j.func = &island_worker_func;
    auto archive = fixed_memory_output_archive(j.data.data(), j.data.size());
    auto worker_intptr = reinterpret_cast<intptr_t>(worker);
    archive(worker_intptr);
    job_dispatcher::global().async(j);

    return entity;
}

entt::entity island_coordinator::merge_islands(const std::unordered_set<entt::entity> &island_entities,
                                               const std::unordered_set<entt::entity> &entities) {
    EDYN_ASSERT(island_entities.size() > 1);
    auto timestamp = m_registry->get<island_timestamp>(*island_entities.begin()).value;
    auto island_entity = create_island(timestamp);

    auto &info = m_island_info_map.at(island_entity);
    auto &builder = info->m_snapshot_builder;

    for (auto entity : entities) {
        // Point container to its new island.
        auto &container = m_registry->get<island_container>(entity);
        container.entities.erase(
            std::remove_if(
                container.entities.begin(), 
                container.entities.end(), 
                [&island_entities] (entt::entity other_island_entity) { 
                    return island_entities.count(other_island_entity) > 0; 
                }),
            container.entities.end());

        if (!vector_contains(container.entities, island_entity)) {
            container.entities.push_back(island_entity);
        }

        // Include all components in snapshot because this is a new entity
        // in that island.
        builder.created(entity);
        builder.maybe_updated(entity, *m_registry, all_components{});
    }

    // Destroy empty islands.
    for (auto other_island_entity : island_entities) {
        auto &info = m_island_info_map.at(other_island_entity);
        info->m_worker->terminate();
        m_island_info_map.erase(other_island_entity);
        m_registry->destroy(other_island_entity);
    }

    return island_entity;
}

void island_coordinator::on_construct_constraint(entt::registry &registry, entt::entity entity) {
    if (m_importing_snapshot) return;

    auto &con = registry.get<constraint>(entity);

    // Initialize constraint.
    std::visit([&] (auto &&c) {
        c.update(solver_stage_value_t<solver_stage::init>{}, entity, con, registry, 0);
    }, con.var);
}

void island_coordinator::on_destroy_constraint(entt::registry &registry, entt::entity entity) {
    if (m_importing_snapshot) return;

    auto &con = registry.get<constraint>(entity);

    // Destroy all constraint rows.
    for (size_t i = 0; i < con.row.size(); ++i) {
        auto row_entity = con.row[i];
        if (registry.valid(row_entity)) {
            registry.destroy(row_entity);
        }
    }
}

void island_coordinator::refresh_dirty_entities() {
    auto view = m_registry->view<island_container, island_node_dirty>();
    view.each([&] (entt::entity entity, island_container &container, island_node_dirty &dirty) {
        for (auto island_entity : container.entities) {
            if (!m_island_info_map.count(island_entity)) continue;
            auto &info = m_island_info_map.at(island_entity);
            auto &builder = info->m_snapshot_builder;
            builder.updated(entity, *m_registry, 
                dirty.indexes.begin(), dirty.indexes.end(), 
                all_components{});

            if (dirty.is_new_entity) {
                builder.created(entity);
            }
        }
    });

    m_registry->clear<island_node_dirty>();
}

void island_coordinator::on_registry_snapshot(entt::entity source_island_entity, const registry_snapshot &snapshot) {
    m_importing_snapshot = true;
    auto &source_info = m_island_info_map.at(source_island_entity);
    snapshot.import(*m_registry, source_info->m_entity_map);
    m_importing_snapshot = false;

    for (auto remote_entity : snapshot.created()) {
        if (!source_info->m_entity_map.has_rem(remote_entity)) continue;
        auto local_entity = source_info->m_entity_map.remloc(remote_entity);
        source_info->m_snapshot_builder.insert_entity_mapping(local_entity);
    }

    if (!snapshot.m_split_connected_components.empty()) {
        auto timestamp = m_registry->get<island_timestamp>(source_island_entity).value;

        const auto &remote_source_entities = snapshot.m_split_connected_components.front();

        for (auto it = std::next(snapshot.m_split_connected_components.begin()); it != snapshot.m_split_connected_components.end(); ++it) {
            auto &entities = *it;
            auto island_entity = create_island(timestamp);
            auto &info = m_island_info_map.at(island_entity);

            // Make containers point to the new island and add entities to the snapshot builder.
            for (auto remote_entity : entities) {
                if (!source_info->m_entity_map.has_rem(remote_entity)) continue;
                auto local_entity = source_info->m_entity_map.remloc(remote_entity);
                if (!m_registry->valid(local_entity)) continue;
                
                auto &container = m_registry->get<island_container>(local_entity);
                
                // Remove source island from container if not there anymore.
                auto not_in_source_island = remote_source_entities.count(remote_entity) == 0;

                if (not_in_source_island) {
                    container.entities.erase(
                        std::remove(
                            container.entities.begin(),
                            container.entities.end(), source_island_entity), 
                        container.entities.end());
                }

                if (!vector_contains(container.entities, island_entity)) {
                    container.entities.push_back(island_entity);
                }

                info->m_snapshot_builder.created(local_entity);
                info->m_snapshot_builder.maybe_updated(local_entity, *m_registry, all_components{});
            }
        }
    }
}

void island_coordinator::sync() {
    for (auto &pair : m_island_info_map) {
        auto &info = pair.second;
        info->sync();
    }
}

void island_coordinator::update() {
    for (auto &pair : m_island_info_map) {
        pair.second->m_message_queue.update();
    }

    refresh_dirty_entities();
    init_new_island_nodes();
    sync();
    validate();
}

void island_coordinator::set_paused(bool paused) {
    m_paused = paused;
    for (auto &pair : m_island_info_map) {
        auto &info = pair.second;
        info->m_message_queue.send<msg::set_paused>(msg::set_paused{paused});

        if (!paused) {
            // If the worker is being unpaused that means it was not running.
            // Thus it is necessary to call `reschedule` to wake it up and
            // process the message sent above.
            info->m_worker->reschedule();
        }
    }
}

void island_coordinator::step_simulation() {
    for (auto &pair : m_island_info_map) {
        auto &info = pair.second;
        info->m_message_queue.send<msg::step_simulation>();
        // The worker is not running while paused, thus it's necessary to call
        // `reschedule` to make it run once. It does not reschedules itself
        // while paused.
        info->m_worker->reschedule();
    }
}

void island_coordinator::validate() {
    const auto &node_view = m_registry->view<const island_node>();

    // All siblings of a node should point back to itself.
    for (entt::entity entity : node_view) {
        auto &node = node_view.get(entity);
        for (auto other : node.entities) {
            auto &other_node = node_view.get(other);
            if (std::find(other_node.entities.begin(), 
                          other_node.entities.end(), 
                          entity) == other_node.entities.end()) {
                EDYN_ASSERT(false);
            }
        }
    }

    auto container_view = m_registry->view<const island_container, const procedural_tag>();

    // All procedural entities should be present in a single island.
    for (entt::entity entity : container_view) {
        auto &container = container_view.get<const island_container>(entity);

        if (container.entities.size() != 1) {
            EDYN_ASSERT(false);
        }

        for (auto island_entity : container.entities) {
            EDYN_ASSERT((m_registry->valid(island_entity)));
        }
    }
}

}
