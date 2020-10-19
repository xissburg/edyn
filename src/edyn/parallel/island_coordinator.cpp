#include "edyn/parallel/island_coordinator.hpp"
#include "edyn/parallel/registry_snapshot.hpp"
#include "edyn/parallel/island_worker.hpp"
#include <entt/entt.hpp>

namespace edyn {

island_coordinator::island_coordinator(entt::registry &registry)
    : m_registry(&registry)
{
    registry.on_construct<island_node>().connect<&island_coordinator::on_construct_island_node>(*this);
    registry.on_destroy<island_node>().connect<&island_coordinator::on_destroy_island_node>(*this);
    registry.on_destroy<island_container>().connect<&island_coordinator::on_destroy_island_container>(*this);
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
    auto &node = registry.get<island_node>(entity);

    // Remove from connected nodes.
    for (auto other : node.entities) {
        // TODO: figure out why `other` could be invalid at this point.
        if (!registry.valid(other)) continue;

        auto &other_node = registry.get<island_node>(other);
        other_node.entities.erase(
            std::remove(
                other_node.entities.begin(),
                other_node.entities.end(), entity), 
            other_node.entities.end());
    }
}

void island_coordinator::on_destroy_island_container(entt::registry &registry, entt::entity entity) {
    auto &container = registry.get<island_container>(entity);

    // Remove from islands.
    for (auto island_entity : container.entities) {
        auto &isle = registry.get<island>(island_entity);
        isle.entities.erase(
            std::remove(
                isle.entities.begin(),
                isle.entities.end(), entity), 
            isle.entities.end());

        auto &info = m_island_info_map.at(island_entity);
        auto builder = registry_snapshot_builder(info->m_entity_map);
        builder.updated(island_entity, isle);
        builder.destroyed(entity);
        info->m_message_queue.send<registry_snapshot>(builder.get_snapshot());
    }
}

void island_coordinator::init_new_island_nodes() {
    while (!m_new_island_nodes.empty()) {
        auto node_entity = m_new_island_nodes.back();
        auto &node = m_registry->get<island_node>(node_entity);

        if (!node.procedural) {
            init_new_non_procedural_island_node(node_entity);
            m_new_island_nodes.pop_back();
            continue;
        }

        // Find connected components to build new islands for procedural entities.
        std::unordered_set<entt::entity> island_entities;
        std::vector<entt::entity> connected;
        std::vector<entt::entity> to_visit;
        to_visit.push_back(node_entity);

        while (!to_visit.empty()) {
            auto entity = to_visit.back();
            to_visit.pop_back();

            // Add to connected component.
            connected.push_back(entity);

            // Remove from main set.
            m_new_island_nodes.erase(
                std::remove(
                    m_new_island_nodes.begin(), 
                    m_new_island_nodes.end(), entity),
                m_new_island_nodes.end());

            // Add related entities to be visited next.
            auto &curr_node = m_registry->get<island_node>(entity);

            for (auto other : curr_node.entities) {
                auto already_visited = std::find(
                    connected.begin(), connected.end(), other) != connected.end();
                if (already_visited) continue;

                // If the other entity is procedural and is already in an island,
                // it means this new procedural entity must join the existing island.
                auto &other_node = m_registry->get<island_node>(other);
                auto &other_container = m_registry->get<island_container>(other);
                if (other_node.procedural && !other_container.entities.empty()) {
                    // Procedural entity must be in only one island.
                    EDYN_ASSERT(other_container.entities.size() == 1);
                    auto island_entity = other_container.entities.front();
                    island_entities.insert(island_entity);
                    // This entity must not be visited because it is already in an
                    // island and thus would be added again, plus it'd be wasteful
                    // to visit the entire island here.
                } else {
                    to_visit.push_back(other);
                }
            }
        }

        // If there's more than one island being connected through the new
        // entities, they must be merged into one.
        if (island_entities.size() > 1) {
            // Merges all into the first island in `island_entities`.
            merge_islands(island_entities);
        } 

        // If there were multiple islands, they have been merged into the first
        // island in `island_entities` thus insert the new entities in it. Or 
        // create a new island if there is none.
        auto island_entity = island_entities.empty() ? create_island() : *island_entities.begin();
        auto &isle = m_registry->get<island>(island_entity);

        // Insert connected entities into island.
        for (auto entity : connected) {
            isle.entities.push_back(entity);
            auto &container = m_registry->get<island_container>(entity);
            container.entities.push_back(island_entity);
        }

        // Send a snapshot containing the new entities to the new island worker.
        auto &info = m_island_info_map.at(island_entity);
        auto builder = registry_snapshot_builder(info->m_entity_map);
        builder.updated<island>(island_entity, isle);
        for (auto entity : connected) {
            builder.maybe_updated(entity, *m_registry, all_components{});
        }
        info->m_message_queue.send<registry_snapshot>(builder.get_snapshot());
    }
}

void island_coordinator::init_new_non_procedural_island_node(entt::entity node_entity) {
    auto &node = m_registry->get<island_node>(node_entity);
    auto &container = m_registry->get<island_container>(node_entity);
    std::unordered_set<entt::entity> island_entities;

    EDYN_ASSERT(!node.procedural);

    // Send new non-procedural entity to island workers of related
    // procedural entities.
    for (auto other : node.entities) {
        auto &other_node = m_registry->get<island_node>(other);
        if (!other_node.procedural) continue;

        auto &other_container = m_registry->get<island_container>(other);
        auto island_entity = other_container.entities.front();
        island_entities.insert(island_entity);
        
        container.entities.push_back(island_entity);
        auto &isle = m_registry->get<island>(island_entity);
        isle.entities.push_back(node_entity);
    }

    for (auto island_entity : island_entities) {
        auto &info = m_island_info_map.at(island_entity);
        auto builder = registry_snapshot_builder(info->m_entity_map);
        builder.updated<island>(island_entity, m_registry->get<island>(island_entity));
        builder.maybe_updated(node_entity, *m_registry, all_components{});
        info->m_message_queue.send<registry_snapshot>(builder.get_snapshot());
    }
}

entt::entity island_coordinator::create_island() {
    auto entity = m_registry->create();
    auto &isle = m_registry->emplace<island>(entity);
    isle.timestamp = (double)performance_counter() / (double)performance_frequency();

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
    builder.updated<island>(entity, isle);
    info->m_message_queue.send<registry_snapshot>(builder.get_snapshot());

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

void island_coordinator::connect_nodes(entt::entity ent0, entt::entity ent1) {
    auto &node0 = m_registry->get<island_node>(ent0);
    auto &node1 = m_registry->get<island_node>(ent1);

    EDYN_ASSERT(std::find(node0.entities.begin(), node0.entities.end(), ent1) == node0.entities.end());
    EDYN_ASSERT(std::find(node1.entities.begin(), node1.entities.end(), ent0) == node1.entities.end());

    node0.entities.push_back(ent1);
    node1.entities.push_back(ent0);

    if (!node0.procedural && !node1.procedural) {
        return;
    }

    if (node0.procedural && node1.procedural) {
        // Find out whether they're in the same island.
        auto &container0 = m_registry->get<island_container>(ent0);
        auto &container1 = m_registry->get<island_container>(ent1);
        auto island_entity0 = container0.entities.front();
        auto island_entity1 = container1.entities.front();

        if (island_entity0 == island_entity1) {
            // Just update nodes.
            auto &info = m_island_info_map.at(island_entity0);
            auto builder = registry_snapshot_builder(info->m_entity_map);
            builder.updated<island_node>(ent0, *m_registry);   
            builder.updated<island_node>(ent1, *m_registry);   
            info->m_message_queue.send<registry_snapshot>(builder.get_snapshot());
        } else {
            merge_islands(std::unordered_set<entt::entity>{island_entity0, island_entity1});
        }
    } else {
        // Add non-procedural entity to island.
        entt::entity procedural_entity, non_procedural_entity;

        if (node0.procedural) {
            procedural_entity = ent0;
            non_procedural_entity = ent1;
        } else {
            procedural_entity = ent1;
            non_procedural_entity = ent0;
        }

        auto &container = m_registry->get<island_container>(procedural_entity);
        auto island_entity = container.entities.front();

        auto &non_procedural_container = m_registry->get<island_container>(non_procedural_entity);
        auto already_in_island = std::find(
            non_procedural_container.entities.begin(),
            non_procedural_container.entities.end(),
            island_entity) != non_procedural_container.entities.end();

        auto &info = m_island_info_map.at(island_entity);
        auto builder = registry_snapshot_builder(info->m_entity_map);
        builder.updated<island_node>(procedural_entity, *m_registry);   

        if (already_in_island) {
            // Just update node.
            builder.updated<island_node>(non_procedural_entity, *m_registry);   
        } else {
            // Add non-procedural to island.
            builder.maybe_updated(non_procedural_entity, *m_registry, all_components{});   
        }

        info->m_message_queue.send<registry_snapshot>(builder.get_snapshot());
    }
}

void island_coordinator::merge_islands(const std::unordered_set<entt::entity> &island_entities) {
    EDYN_ASSERT(island_entities.size() > 1);
    auto island_entity = *island_entities.begin();
    auto &isle = m_registry->get<island>(island_entity);
    std::vector<entt::entity> other_entities;

    for (auto it = std::next(island_entities.begin()); it != island_entities.end(); ++it) {
        auto &other_isle = m_registry->get<island>(*it);
        for (auto ent : other_isle.entities) {
            other_entities.push_back(ent);
            isle.entities.push_back(ent);
            auto &container = m_registry->get<island_container>(ent);
            container.entities.clear();
            container.entities.push_back(island_entity);
        }
    }

    // Send snapshot containing moved entities to the first island.
    auto &info = m_island_info_map.at(island_entity);
    auto builder = registry_snapshot_builder(info->m_entity_map);
    builder.updated<island>(island_entity, isle);

    for (auto ent : other_entities) {
        builder.maybe_updated(ent, *m_registry, all_components{});
    }

    info->m_message_queue.send<registry_snapshot>(builder.get_snapshot());

    // Destroy empty islands.
    for (auto it = std::next(island_entities.begin()); it != island_entities.end(); ++it) {
        auto other_isle_entity = *it;
        auto &info1 = m_island_info_map.at(other_isle_entity);
        info1->m_worker->terminate();
        m_island_info_map.erase(other_isle_entity);
        m_registry->destroy(other_isle_entity);
    }
}

void island_coordinator::refresh_dirty_entities() {
    auto view = m_registry->view<island_container, island_node_dirty>();
    view.each([&] (entt::entity entity, island_container &container, island_node_dirty &dirty) {
        for (auto island_entity : container.entities) {
            auto &info = m_island_info_map.at(island_entity);
            auto builder = registry_snapshot_builder(info->m_entity_map);
            builder.updated(entity, *m_registry, 
                dirty.indexes.begin(), dirty.indexes.end(), 
                all_components{});
            info->m_message_queue.send<registry_snapshot>(builder.get_snapshot());
        }
    });
    m_registry->clear<island_node_dirty>();
}

void island_coordinator::on_registry_snapshot(entt::entity island_entity, const registry_snapshot &snapshot) {
    m_importing_snapshot = true;
    auto &info = m_island_info_map.at(island_entity);
    snapshot.import(*m_registry, info->m_entity_map);
    m_importing_snapshot = false;
}

void island_coordinator::update() {
    for (auto &pair : m_island_info_map) {
        pair.second->m_message_queue.update();
    }

    refresh_dirty_entities();
    init_new_island_nodes();
}

void island_coordinator::set_paused(bool paused) {
    for (auto &pair : m_island_info_map) {
        pair.second->m_message_queue.send<msg::set_paused>(msg::set_paused{paused});
    }
}

void island_coordinator::step_simulation() {
    for (auto &pair : m_island_info_map) {
        pair.second->m_message_queue.send<msg::step_simulation>();
    }
}

}
