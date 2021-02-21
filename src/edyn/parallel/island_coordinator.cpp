#include "edyn/parallel/island_coordinator.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/constraints/contact_constraint.hpp"
#include "edyn/parallel/island_delta.hpp"
#include "edyn/parallel/island_worker.hpp"
#include "edyn/comp/dirty.hpp"
#include "edyn/time/time.hpp"
#include "edyn/parallel/entity_graph.hpp"
#include "edyn/comp/graph_node.hpp"
#include "edyn/comp/graph_edge.hpp"
#include <entt/entt.hpp>

namespace edyn {

island_coordinator::island_coordinator(entt::registry &registry)
    : m_registry(&registry)
{
    registry.on_construct<graph_node>().connect<&island_coordinator::on_construct_graph_node>(*this);
    registry.on_destroy<graph_node>().connect<&island_coordinator::on_destroy_graph_node>(*this);
    registry.on_construct<graph_edge>().connect<&island_coordinator::on_construct_graph_edge>(*this);
    registry.on_destroy<graph_edge>().connect<&island_coordinator::on_destroy_graph_edge>(*this);

    registry.on_destroy<island_container>().connect<&island_coordinator::on_destroy_island_container>(*this);

    registry.on_construct<constraint>().connect<&island_coordinator::on_construct_constraint>(*this);
}

island_coordinator::~island_coordinator() {
    for (auto &pair : m_island_ctx_map) {
        auto &ctx = pair.second;
        ctx->terminate();
    }
}

void island_coordinator::on_construct_graph_node(entt::registry &registry, entt::entity entity) {
    if (!m_importing_delta) {
        m_new_graph_nodes.push_back(entity);
    }
}

void island_coordinator::on_construct_graph_edge(entt::registry &registry, entt::entity entity) {
    if (!m_importing_delta) {
        m_new_graph_edges.push_back(entity);
    }
}

void island_coordinator::on_destroy_graph_node(entt::registry &registry, entt::entity entity) {
    auto &node = registry.get<graph_node>(entity);
    registry.ctx<entity_graph>().remove_node(node.node_index);
}

void island_coordinator::on_destroy_graph_edge(entt::registry &registry, entt::entity entity) {
    auto &edge = registry.get<graph_edge>(entity);
    registry.ctx<entity_graph>().remove_edge(edge.edge_index);
}

void island_coordinator::on_destroy_island_container(entt::registry &registry, entt::entity entity) {
    auto &container = registry.get<island_container>(entity);

    // Remove from islands.
    for (auto island_entity : container.entities) {
        auto &ctx = m_island_ctx_map.at(island_entity);
        ctx->m_nodes.erase(entity);
        ctx->m_edges.erase(entity);

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
    if (m_new_graph_nodes.empty()) return;

    auto node_view = m_registry->view<graph_node>();
    std::vector<entity_graph::index_type> procedural_node_indices;

    for (auto entity : m_new_graph_nodes) {
        if (m_registry->has<procedural_tag>(entity)) {
            auto &node = node_view.get(entity);
            procedural_node_indices.push_back(node.node_index);
        } else {
            init_new_non_procedural_island_node(entity);
        }
    }

    m_new_graph_nodes.clear();
    std::vector<entt::entity> connected_nodes;
    std::vector<entt::entity> connected_edges;
    entity_set island_entities;
    auto &graph = m_registry->ctx<entity_graph>();

    graph.reach(
        procedural_node_indices.begin(), 
        procedural_node_indices.end(),
        [&] (entity_graph::index_type node_index, entity_graph::index_type adj_index) { // visitFunc
            auto entity = graph.node_entity(node_index);
            connected_nodes.push_back(entity);

            graph.visit_edges(adj_index, [&connected_edges] (entt::entity edge_entity) {
                connected_edges.push_back(edge_entity);
            });
        },
        [&] (entity_graph::index_type node_index) { // shouldVisitFunc
            auto other_entity = graph.node_entity(node_index);

            if (m_registry->has<procedural_tag>(other_entity)) {
                // Collect islands involved in this connected component.
                auto &other_container = m_registry->get<island_container>(other_entity);
                if (!other_container.entities.empty()) {
                    // Procedural entity must be in only one island.
                    EDYN_ASSERT(other_container.entities.size() == 1);
                    auto island_entity = *other_container.entities.begin();
                    island_entities.insert(island_entity);
                    // This entity must not be visited because it is already in an
                    // island. This prevents visiting the entire island which 
                    // would be wasteful.
                    return false;
                } else {
                    return true;
                }
            } else {
                // Non-procedural nodes must not be visited because they do
                // not provide a path to connect other nodes through itself.
                connected_nodes.push_back(other_entity);
                return false;
            }
        },
        [&] () { // connectedComponentFunc
            if (island_entities.empty()) {
                const auto timestamp = (double)performance_counter() / (double)performance_frequency();
                auto island_entity = create_island(timestamp, false);
                insert_to_island(island_entity, connected_nodes, connected_edges);
            } else if (island_entities.size() == 1) {
                auto island_entity = *island_entities.begin();
                insert_to_island(island_entity, connected_nodes, connected_edges);
            } else {
                merge_islands(island_entities, connected_nodes, connected_edges);
            }

            connected_nodes.clear();
            connected_edges.clear();
            island_entities.clear();
        });
}

void island_coordinator::init_new_non_procedural_island_node(entt::entity node_entity) {
    auto &node = m_registry->get<graph_node>(node_entity);
    auto &container = m_registry->get<island_container>(node_entity);

    EDYN_ASSERT(!(m_registry->has<procedural_tag>(node_entity)));

    // Add new non-procedural entity to islands of neighboring procedural entities.
    m_registry->ctx<entity_graph>().visit_neighbors(node.node_index, [&] (entt::entity other) {
        if (!m_registry->has<procedural_tag>(other)) return;

        auto &other_container = m_registry->get<island_container>(other);
        if (other_container.entities.empty()) return;

        auto island_entity = *other_container.entities.begin();
        auto &ctx = m_island_ctx_map.at(island_entity);
        ctx->m_nodes.insert(node_entity);

        if (!container.entities.count(island_entity)) {
            container.entities.insert(island_entity);
            ctx->m_delta_builder->created(node_entity);
            ctx->m_delta_builder->created_all(node_entity, *m_registry);
        }
    });
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
    ctx->split_island_sink().connect<&island_coordinator::on_split_island>(*this);

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

void island_coordinator::insert_to_island(entt::entity island_entity, 
                                          const std::vector<entt::entity> &nodes,
                                          const std::vector<entt::entity> &edges) {

    auto &ctx = m_island_ctx_map.at(island_entity);
    ctx->m_nodes.insert(nodes.begin(), nodes.end());
    ctx->m_edges.insert(edges.begin(), edges.end());
    
    auto builder = make_island_delta_builder(ctx->m_entity_map);

    for (auto entity : nodes) {
        // Assign island to containers.
        auto &container = m_registry->get<island_container>(entity);

        // Non-procedural entity might already be in this island.
        if (container.entities.count(island_entity)) {
            builder->updated_all(entity, *m_registry);
        } else {
            container.entities.insert(island_entity);
            // Add new entities to the delta builder.
            builder->created(entity);
            builder->created_all(entity, *m_registry);
        }
    }

    for (auto entity : edges) {
        // Assign island to containers.
        auto &container = m_registry->get<island_container>(entity);
        container.entities.insert(island_entity);
        // Add new entities to the delta builder.
        builder->created(entity);
        builder->created_all(entity, *m_registry);

        // Add child entities.
        if (auto *manifold = m_registry->try_get<contact_manifold>(entity); manifold) {
            auto num_points = manifold->num_points();
            for (size_t i = 0; i < num_points; ++i) {
                auto point_entity = manifold->point[i];
                builder->created(point_entity);
                builder->created_all(point_entity, *m_registry);

                auto &con = m_registry->get<constraint>(point_entity);
                auto num_rows = con.num_rows();
                for (size_t j = 0; j < num_rows; ++j) {
                    auto row_entity = con.row[j];
                    builder->created(row_entity);
                    builder->created_all(row_entity, *m_registry);
                }
            }
        } else {
            auto &con = m_registry->get<constraint>(entity);
            auto num_rows = con.num_rows();
            for (size_t i = 0; i < num_rows; ++i) {
                auto row_entity = con.row[i];
                builder->created(row_entity);
                builder->created_all(row_entity, *m_registry);
            }
        }
    }

    auto delta = builder->finish();
    ctx->send<island_delta>(std::move(delta));
}

entt::entity island_coordinator::merge_islands(const entity_set &island_entities,
                                               const std::vector<entt::entity> &new_nodes,
                                               const std::vector<entt::entity> &new_edges) {
    EDYN_ASSERT(island_entities.size() > 1);

    // Pick biggest island and move the other entities into it.
    entt::entity island_entity;
    size_t biggest_size = 0;

    for (auto entity : island_entities) {
        auto &ctx = m_island_ctx_map.at(entity);
        auto size = ctx->m_nodes.size() + ctx->m_edges.size();

        if (size > biggest_size) {
            biggest_size = size;
            island_entity = entity;
        }
    }

    auto other_island_entities = island_entities;
    other_island_entities.erase(island_entity);

    auto all_nodes = new_nodes;
    auto all_edges = new_edges;

    for (auto other_island_entity : other_island_entities) {
        auto &ctx = m_island_ctx_map.at(other_island_entity);
        all_nodes.insert(all_nodes.end(), ctx->m_nodes.begin(), ctx->m_nodes.end());
        all_edges.insert(all_edges.end(), ctx->m_edges.begin(), ctx->m_edges.end());
    }

    // Entity might be coming from a sleeping island. Remove `sleeping_tag`s
    // since the island is supposed to be awake after a merge.
    for (auto entity : all_nodes) {
        m_registry->remove_if_exists<sleeping_tag>(entity);
    }
    for (auto entity : all_edges) {
        m_registry->remove_if_exists<sleeping_tag>(entity);
    }

    insert_to_island(island_entity, all_nodes, all_edges);

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

    // Insert entity mappings for new entities into the current delta.
    for (auto remote_entity : delta.created_entities()) {
        if (!source_ctx->m_entity_map.has_rem(remote_entity)) continue;
        auto local_entity = source_ctx->m_entity_map.remloc(remote_entity);
        source_ctx->m_delta_builder->insert_entity_mapping(local_entity);
    }

    // Assign `island_container` to entities created in the island worker and
    // associate them with the source island entity.
    for (auto entity : delta.created_entities()) {
        auto &container = m_registry->emplace<island_container>(entity);
        container.entities.insert(source_island_entity);
    }

    // Insert nodes in the graph for each rigid body.
    auto &graph = m_registry->ctx<entity_graph>();
    auto insert_node = [&] (entt::entity entity, auto &) {
        auto node_index = graph.insert_node(entity);
        m_registry->emplace<graph_node>(entity, node_index);
    };

    delta.created_for_each<dynamic_tag>(insert_node);
    delta.created_for_each<static_tag>(insert_node);
    delta.created_for_each<kinematic_tag>(insert_node);

    auto node_view = m_registry->view<graph_node>();

    // Insert edges in the graph for contact manifolds.
    delta.created_for_each<contact_manifold>([&] (entt::entity entity, const contact_manifold &manifold) {
        auto &node0 = node_view.get(manifold.body[0]);
        auto &node1 = node_view.get(manifold.body[1]);
        auto edge_index = graph.insert_edge(entity, node0.node_index, node1.node_index);
        m_registry->emplace<graph_edge>(entity, edge_index);
    });

    // Insert edges in the graph for constraints (except contact constraints).
    delta.created_for_each<constraint>([&] (entt::entity entity, const constraint &con) {
        // Contact constraints are not added as edges to the graph.
        // The contact manifold which owns them is added instead.
        if (std::holds_alternative<contact_constraint>(con.var)) return;

        auto &node0 = node_view.get(con.body[0]);
        auto &node1 = node_view.get(con.body[1]);
        auto edge_index = graph.insert_edge(entity, node0.node_index, node1.node_index);
        m_registry->emplace<graph_edge>(entity, edge_index);
    });

    m_importing_delta = false;
}

void island_coordinator::on_split_island(entt::entity source_island_entity, const msg::split_island &) {
    auto &source_ctx = m_island_ctx_map.at(source_island_entity);
    if (!source_ctx->m_pending_split) {
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
    auto node_view = m_registry->view<graph_node>();
    std::vector<entity_graph::index_type> node_indices;

    for (auto entity : split_ctx->m_nodes) {
        auto &node = node_view.get(entity);
        node_indices.push_back(node.node_index);
    }

    auto &graph = m_registry->ctx<entity_graph>();

    auto connected_components = graph.connected_components(
        node_indices.begin(), node_indices.end(), 
        [&] (entt::entity current, entt::entity neighbor) {
            // Non-procedural nodes should only connect non-procedural nodes
            // because a procedural node cannot affect another through a
            // non-procedural node.
            auto is_procedural = m_registry->has<procedural_tag>(current);
            return is_procedural || (!is_procedural && !m_registry->has<procedural_tag>(neighbor));
        });

    EDYN_ASSERT(!connected_components.empty());

    if (connected_components.size() == 1) return;

    auto timestamp = m_registry->get<island_timestamp>(split_island_entity).value;
    bool sleeping = m_registry->has<sleeping_tag>(split_island_entity);
    auto container_view = m_registry->view<island_container>();

    for (auto &connected : connected_components) {
        bool contains_procedural = false;

        // Remove deceased island from containers.
        for (auto entity : connected.nodes) {
            auto &container = container_view.get(entity);
            container.entities.erase(split_island_entity);

            if (m_registry->has<procedural_tag>(entity)) {
                contains_procedural = true;
            }
        }

        for (auto entity : connected.edges) {
            auto &container = container_view.get(entity);
            container.entities.erase(split_island_entity);
        }

        // Do not create a new island if this connected component does not
        // contain any procedural node. 
        if (!contains_procedural) continue;

        auto island_entity = create_island(timestamp, sleeping);
        insert_to_island(island_entity, connected.nodes, connected.edges);
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
