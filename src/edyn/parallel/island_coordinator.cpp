#include "edyn/parallel/island_coordinator.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/parallel/island_delta.hpp"
#include "edyn/parallel/island_worker.hpp"
#include "edyn/parallel/island_topology.hpp"
#include "edyn/util/island_util.hpp"
#include "edyn/comp/dirty.hpp"
#include "edyn/time/time.hpp"
#include <entt/entt.hpp>

namespace edyn {

island_coordinator::island_coordinator(entt::registry &registry)
    : m_registry(&registry)
{
    registry.on_destroy<island_node_parent>().connect<&island_coordinator::on_destroy_island_node_parent>(*this);

    registry.on_construct<island_node>().connect<&island_coordinator::on_construct_island_node>(*this);
    registry.on_destroy<island_node>().connect<&island_coordinator::on_destroy_island_node>(*this);

    registry.on_construct<island_container>().connect<&island_coordinator::on_construct_island_container>(*this);
    registry.on_destroy<island_container>().connect<&island_coordinator::on_destroy_island_container>(*this);

    registry.on_construct<constraint>().connect<&island_coordinator::on_construct_constraint>(*this);
}

island_coordinator::~island_coordinator() {
    for (auto &pair : m_island_ctx_map) {
        auto &ctx = pair.second;
        ctx->terminate();
    }
}

void island_coordinator::on_destroy_island_node_parent(entt::registry &registry, entt::entity entity) {
    if (m_importing_delta) return;
    edyn::on_destroy_island_node_parent(registry, entity);
}

void island_coordinator::on_construct_island_node(entt::registry &registry, entt::entity entity) {
    if (m_importing_delta) return;

    m_new_island_nodes.push_back(entity);
}

void island_coordinator::on_destroy_island_node(entt::registry &registry, entt::entity entity) {
    if (m_importing_delta) return;

    auto &node = registry.get<island_node>(entity);

    // Remove from connected nodes.
    for (auto other : node.entities) {
        auto &other_node = registry.get<island_node>(other);
        other_node.entities.erase( entity);
        registry.get_or_emplace<dirty>(other).updated<island_node>();
    }
}

void island_coordinator::on_construct_island_container(entt::registry &registry, entt::entity entity) {
    auto &container = registry.get<island_container>(entity);

    for (auto island_entity : container.entities) {
        auto &ctx = m_island_ctx_map.at(island_entity);
        ctx->m_entities.insert(entity);
    }
}

void island_coordinator::on_destroy_island_container(entt::registry &registry, entt::entity entity) {
    auto &container = registry.get<island_container>(entity);

    // Remove from islands.
    for (auto island_entity : container.entities) {
        auto &ctx = m_island_ctx_map.at(island_entity);
        EDYN_ASSERT(ctx->m_entities.count(entity));
        ctx->m_entities.erase(entity);
        if (!m_importing_delta)  {
            ctx->m_delta_builder->destroyed(entity);
        }
    }
}

void island_coordinator::on_construct_constraint(entt::registry &registry, entt::entity entity) {
    if (m_importing_delta) return;

    auto &con = registry.get<constraint>(entity);

    // Initialize constraint.
    std::visit([&] (auto &&c) {
        c.update(solver_stage_value_t<solver_stage::init>{}, entity, con, registry, 0);
    }, con.var);
}

void island_coordinator::init_new_island_nodes() {
    if (m_new_island_nodes.empty()) return;
    
    entity_set procedural_node_entities;

    for (auto entity : m_new_island_nodes) {
        if (m_registry->has<procedural_tag>(entity)) {
            procedural_node_entities.insert(entity);
        } else {
            init_new_non_procedural_island_node(entity);
        }
    }

    m_new_island_nodes.clear();

    while (!procedural_node_entities.empty()) {
        // Find connected components to build new islands for procedural entities.
        entity_set island_entities;
        entity_set connected;
        std::vector<entt::entity> to_visit;

        auto node_entity = *procedural_node_entities.begin();
        to_visit.push_back(node_entity);

        while (!to_visit.empty()) {
            auto curr_entity = to_visit.back();
            to_visit.pop_back();

            // Add to connected component.
            connected.insert(curr_entity);

            if (m_registry->has<island_node_parent>(curr_entity)) {
                // Add children.
                auto children = get_island_node_children(*m_registry, curr_entity);
                connected.insert(children.begin(), children.end());
            }

            // Remove from main set.
            procedural_node_entities.erase(curr_entity);

            // Add related entities to be visited next.
            auto &curr_node = m_registry->get<island_node>(curr_entity);

            for (auto other_entity : curr_node.entities) {
                auto already_visited = connected.count(other_entity);
                if (already_visited) continue;

                if (m_registry->has<procedural_tag>(other_entity)) {
                    // Collect islands involved in this connected component.
                    auto &other_container = m_registry->get<island_container>(other_entity);
                    if (!other_container.entities.empty()) {
                        // Procedural entity must be in only one island.
                        EDYN_ASSERT(other_container.entities.size() == 1);
                        auto island_entity = *other_container.entities.begin();
                        island_entities.insert(island_entity);
                        // This entity must not be visited because it is already in an
                        // island and thus would be added again, plus it prevents 
                        // visiting the entire island which could be quite a ride.
                    } else {
                        to_visit.push_back(other_entity);
                    }
                } else {
                    // Non-procedural nodes must not be visited because they do
                    // not provide a path to connect other nodes through itself.
                    connected.insert(other_entity);
                }
            }
        }

        if (island_entities.empty()) {
            const auto timestamp = (double)performance_counter() / (double)performance_frequency();
            auto island_entity = create_island(timestamp, false);

            auto &ctx = m_island_ctx_map.at(island_entity);
            ctx->m_entities = connected;
            auto builder = make_island_delta_builder(ctx->m_entity_map);

            for (auto entity : connected) {
                // Assign island to containers.
                auto &container = m_registry->get<island_container>(entity);
                container.entities.insert(island_entity);
                // Add new entities to the delta builder.
                builder->created(entity);
                builder->created_all(entity, *m_registry);
            }

            auto delta = builder->finish();
            ctx->send<island_delta>(std::move(delta));
        } else if (island_entities.size() == 1) {
            auto island_entity = *island_entities.begin();
            
            auto &ctx = m_island_ctx_map.at(island_entity);
            ctx->m_entities.insert(connected.begin(), connected.end());

            for (auto entity : connected) {
                // Assign island to containers.
                auto &container = m_registry->get<island_container>(entity);
                if (!container.entities.count(island_entity)) {
                    container.entities.insert(island_entity);
                    // Add new entities to the delta builder.
                    ctx->m_delta_builder->created(entity);
                    ctx->m_delta_builder->created_all(entity, *m_registry);
                } else {
                    ctx->m_delta_builder->updated_all(entity, *m_registry);
                }
            }
        } else {
            merge_islands(island_entities, connected);
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

        auto island_entity = *other_container.entities.begin();
        auto &ctx = m_island_ctx_map.at(island_entity);
        ctx->m_entities.insert(node_entity);

        if (!container.entities.count(island_entity)) {
            container.entities.insert(island_entity);
            ctx->m_delta_builder->created(node_entity);
            ctx->m_delta_builder->created_all(node_entity, *m_registry);
        }
    }
}

entt::entity island_coordinator::create_island(double timestamp, bool sleeping) {
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
    auto ctx = std::make_unique<island_worker_context>(entity, worker, message_queue_in_out(isle_queue_input, main_queue_output));
    
    // Register to receive delta.
    ctx->island_delta_sink().connect<&island_coordinator::on_island_delta>(*this);
    ctx->island_topology_sink().connect<&island_coordinator::on_island_topology>(*this);

    // Send over a delta containing this island entity to the island worker
    // before it even starts.
    auto builder = make_island_delta_builder(ctx->m_entity_map);
    builder->created(entity);
    builder->created(entity, isle_time);

    if (sleeping) {
        m_registry->emplace<sleeping_tag>(entity);
        builder->created(entity, sleeping_tag{});
    }

    auto delta = builder->finish();
    ctx->send<island_delta>(std::move(delta));

    if (m_paused) {
        ctx->send<msg::set_paused>(true);
    }

    m_island_ctx_map.emplace(entity, std::move(ctx));

    return entity;
}

entt::entity island_coordinator::merge_islands(const entity_set &island_entities,
                                               const entity_set &new_entities) {
    EDYN_ASSERT(island_entities.size() > 1);

    // Pick biggest island and move the other entities into it.
    entt::entity island_entity;
    size_t biggest_size = 0;

    for (auto entity : island_entities) {
        auto &ctx = m_island_ctx_map.at(entity);
        
        if (ctx->m_entities.size() > biggest_size) {
            biggest_size = ctx->m_entities.size();
            island_entity = entity;
        }
    }

    auto other_island_entities = island_entities;
    other_island_entities.erase(island_entity);

    auto all_entities = new_entities;

    for (auto other_island_entity : other_island_entities) {
        auto &ctx = m_island_ctx_map.at(other_island_entity);
        all_entities.insert(ctx->m_entities.begin(), ctx->m_entities.end());
    }

    auto &ctx = m_island_ctx_map.at(island_entity);
    auto &builder = ctx->m_delta_builder;

    for (auto entity : all_entities) {
        // Point container to its new island.
        auto &container = m_registry->get<island_container>(entity);

        for (auto other_island_entity : other_island_entities) {
            container.entities.erase(other_island_entity);
        }

        container.entities.insert(island_entity);

        if (m_registry->has<procedural_tag>(entity)) {
            EDYN_ASSERT(container.entities.size() <= 1);
        }

        // Entity might be coming from a sleeping island. Remove `sleeping_tag`s
        // since the island is supposed to be awake after a merge.
        m_registry->remove_if_exists<sleeping_tag>(entity);

        if (!ctx->m_entities.count(entity)) {
            // Include all components in delta because this is a new entity
            // in the selected island.
            builder->created(entity);
            builder->created_all(entity, *m_registry);
        } else {
            builder->updated_all(entity, *m_registry);
        }
    }

    ctx->m_entities.insert(all_entities.begin(), all_entities.end());

    // Destroy empty islands.
    for (auto other_island_entity : other_island_entities) {
        auto &ctx = m_island_ctx_map.at(other_island_entity);
        ctx->terminate();
        m_island_ctx_map.erase(other_island_entity);
        m_registry->destroy(other_island_entity);
    }

    // Prevents glitch where entities are moved into an island that was sleeping
    // and thus its timestamp is outdated.
    if (m_registry->has<sleeping_tag>(island_entity)) {
        auto &isle_timestamp = m_registry->get<island_timestamp>(island_entity);
        isle_timestamp.value = (double)performance_counter() / (double)performance_frequency();
    }

    return island_entity;
}

void island_coordinator::refresh_dirty_entities() {
    auto view = m_registry->view<island_container, dirty>();
    view.each([&] (entt::entity entity, island_container &container, dirty &dirty) {
        std::vector<entt::entity> island_entities;
        
        if (dirty.island_entities.empty()) {
            island_entities.insert(island_entities.end(), container.entities.begin(), container.entities.end());
        } else {
            island_entities = dirty.island_entities;
        }

        for (auto island_entity : island_entities) {
            if (!m_island_ctx_map.count(island_entity)) continue;
            auto &ctx = m_island_ctx_map.at(island_entity);
            auto &builder = ctx->m_delta_builder;

            if (dirty.is_new_entity) {
                builder->created(entity);
            }

            builder->created(entity, *m_registry, 
                dirty.created_indexes.begin(), dirty.created_indexes.end());
            builder->updated(entity, *m_registry, 
                dirty.updated_indexes.begin(), dirty.updated_indexes.end());
            builder->destroyed(entity, 
                dirty.destroyed_indexes.begin(), dirty.destroyed_indexes.end());
        }
    });

    m_registry->clear<dirty>();
}

void island_coordinator::on_island_delta(entt::entity source_island_entity, const island_delta &delta) {
    m_importing_delta = true;
    auto &source_ctx = m_island_ctx_map.at(source_island_entity);
    delta.import(*m_registry, source_ctx->m_entity_map);
    m_importing_delta = false;

    // Insert entity mappings for new entities into the current delta.
    for (auto remote_entity : delta.created_entities()) {
        if (!source_ctx->m_entity_map.has_rem(remote_entity)) continue;
        auto local_entity = source_ctx->m_entity_map.remloc(remote_entity);
        source_ctx->m_delta_builder->insert_entity_mapping(local_entity);
    }
}

void island_coordinator::on_island_topology(entt::entity source_island_entity, const island_topology &topology) {
    // TODO: Use a different condition to split islands, e.g. calculate variance
    // in size of connected components and only split if there isn't much variance.
    auto &source_ctx = m_island_ctx_map.at(source_island_entity);
    if (source_ctx->m_pending_split) {
        if (topology.component_sizes.size() <= 1) {
            // Cancel split.
            source_ctx->m_pending_split = false;
        }
    } else if (topology.component_sizes.size() > 1) {
        source_ctx->m_split_timestamp = (double)performance_counter() / (double)performance_frequency();
        source_ctx->m_pending_split = true;
    }
}

void island_coordinator::split_islands() {
    std::vector<entt::entity> islands_to_split;
    auto time = (double)performance_counter() / (double)performance_frequency();

    for (auto &pair : m_island_ctx_map) {
        auto &ctx = pair.second;
        if (!ctx->m_pending_split) continue;

        auto dt = time - ctx->m_split_timestamp;
        if (dt > m_island_split_delay) {
            ctx->m_pending_split = false;
            islands_to_split.push_back(pair.first);
        }
    }

    for (auto split_island_entity : islands_to_split) {
        split_island(split_island_entity);
    }
}

void island_coordinator::split_island(entt::entity split_island_entity) {
    if (m_island_ctx_map.count(split_island_entity) == 0) return;

    auto &split_ctx = m_island_ctx_map.at(split_island_entity);
    entity_set node_entities;
    node_entities.reserve(split_ctx->m_entities.size());

    for (auto entity : split_ctx->m_entities) {
        if (m_registry->has<island_node>(entity)) {
            node_entities.insert(entity);
        }
    }

    auto node_view = m_registry->view<island_node>();
    std::vector<entity_set> connected_components;

    while (!node_entities.empty()) {
        entity_set connected;
        std::vector<entt::entity> to_visit;

        auto node_entity = *node_entities.begin();
        to_visit.push_back(node_entity);

        while (!to_visit.empty()) {
            auto entity = to_visit.back();
            to_visit.pop_back();

            // Add to connected component.
            connected.insert(entity);

            if (m_registry->has<island_node_parent>(entity)) {
                // Add children.
                auto children = get_island_node_children(*m_registry, entity);
                connected.insert(children.begin(), children.end());
            }

            // Remove from main set.
            node_entities.erase(entity);

            // Add related entities to be visited next.
            auto &curr_node = node_view.get(entity);
            auto is_procedural = m_registry->has<procedural_tag>(entity);

            for (auto other : curr_node.entities) {
                auto already_visited = connected.count(other);
                if (already_visited) continue;

                // Non-procedural nodes should only connect non-procedural nodes
                // because a procedural node cannot affect another through a
                // non-procedural node.
                if (is_procedural || (!is_procedural && !m_registry->has<procedural_tag>(other))) {
                    to_visit.push_back(other);
                }
            }
        }

        EDYN_ASSERT(!connected.empty());
        connected_components.push_back(connected);
    }

    EDYN_ASSERT(!connected_components.empty());

    if (connected_components.size() == 1) return;

    auto timestamp = m_registry->get<island_timestamp>(split_island_entity).value;
    bool sleeping = m_registry->has<sleeping_tag>(split_island_entity);
    auto container_view = m_registry->view<island_container>();

    for (auto &connected : connected_components) {
        bool contains_procedural = false;

        // Remove deceased island from containers.
        for (auto entity : connected) {
            auto &container = container_view.get(entity);
            container.entities.erase(split_island_entity);

            if (m_registry->has<procedural_tag>(entity)) {
                contains_procedural = true;
            }
        }

        // Do not create a new island if this connected component does not
        // contain any procedural node. 
        if (!contains_procedural) continue;

        auto island_entity = create_island(timestamp, sleeping);
        auto &ctx = m_island_ctx_map.at(island_entity);
        ctx->m_entities = connected;

        // Make containers point to the new island and add entities to the delta builder.
        auto builder = make_island_delta_builder(ctx->m_entity_map);

        for (auto entity : connected) {
            auto &container = container_view.get(entity);
            container.entities.insert(island_entity);

            if (m_registry->has<procedural_tag>(entity)) {
                EDYN_ASSERT(container.entities.size() <= 1);
            }

            builder->created(entity);
            builder->created_all(entity, *m_registry);
        }

        auto delta = builder->finish();
        ctx->send<island_delta>(std::move(delta));
    }

    split_ctx->terminate();
    m_island_ctx_map.erase(split_island_entity);
    m_registry->destroy(split_island_entity);
}

void island_coordinator::sync() {
    for (auto &pair : m_island_ctx_map) {
        auto island_entity = pair.first;
        auto &ctx = pair.second;

        if (!ctx->delta_empty()) {
            ctx->send_delta();

            if (m_registry->has<sleeping_tag>(island_entity)) {
                ctx->send<msg::wake_up_island>();
            }
        }

        ctx->flush();
    }
}

void island_coordinator::update() {
    for (auto &pair : m_island_ctx_map) {
        pair.second->read_messages();
    }

    init_new_island_nodes();
    split_islands();
    refresh_dirty_entities();
    sync();

#ifdef DEBUG
    validate();
#endif
}

void island_coordinator::set_paused(bool paused) {
    m_paused = paused;
    for (auto &pair : m_island_ctx_map) {
        auto &ctx = pair.second;
        ctx->send<msg::set_paused>(paused);
    }
}

void island_coordinator::step_simulation() {
    for (auto &pair : m_island_ctx_map) {
        auto island_entity = pair.first;
        if (m_registry->has<sleeping_tag>(island_entity)) continue;

        auto &ctx = pair.second;
        ctx->send<msg::step_simulation>();
    }
}

void island_coordinator::validate() {
    const auto &node_view = m_registry->view<const island_node>();

    // All siblings of a node should point back to itself.
    for (entt::entity entity : node_view) {
        auto &node = node_view.get(entity);
        for (auto other : node.entities) {
            auto &other_node = node_view.get(other);
            EDYN_ASSERT(other_node.entities.count(entity));
        }
    }

    auto container_view = m_registry->view<const island_container, const procedural_tag>();

    // All procedural entities should be present in a single island.
    for (entt::entity entity : container_view) {
        auto &container = container_view.get<const island_container>(entity);

        EDYN_ASSERT(container.entities.size() == 1);

        for (auto island_entity : container.entities) {
            EDYN_ASSERT((m_registry->valid(island_entity)));
        }
    }
}

}
