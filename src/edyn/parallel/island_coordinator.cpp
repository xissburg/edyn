#include "edyn/parallel/island_coordinator.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/collision/contact_point.hpp"
#include "edyn/collision/dynamic_tree.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/comp/shape_index.hpp"
#include "edyn/parallel/message.hpp"
#include "edyn/shapes/shapes.hpp"
#include "edyn/config/config.h"
#include "edyn/constraints/constraint.hpp"
#include "edyn/constraints/contact_constraint.hpp"
#include "edyn/parallel/island_worker.hpp"
#include "edyn/comp/dirty.hpp"
#include "edyn/time/time.hpp"
#include "edyn/parallel/entity_graph.hpp"
#include "edyn/comp/graph_node.hpp"
#include "edyn/comp/graph_edge.hpp"
#include "edyn/util/vector.hpp"
#include "edyn/util/registry_operation.hpp"
#include "edyn/context/settings.hpp"
#include "edyn/dynamics/material_mixing.hpp"
#include <entt/entity/registry.hpp>
#include <set>

namespace edyn {

island_coordinator::island_coordinator(entt::registry &registry)
    : m_registry(&registry)
{
    registry.on_construct<graph_node>().connect<&island_coordinator::on_construct_graph_node>(*this);
    registry.on_destroy<graph_node>().connect<&island_coordinator::on_destroy_graph_node>(*this);
    registry.on_construct<graph_edge>().connect<&island_coordinator::on_construct_graph_edge>(*this);
    registry.on_destroy<graph_edge>().connect<&island_coordinator::on_destroy_graph_edge>(*this);

    registry.on_destroy<island_resident>().connect<&island_coordinator::on_destroy_island_resident>(*this);
    registry.on_destroy<multi_island_resident>().connect<&island_coordinator::on_destroy_multi_island_resident>(*this);

    registry.on_destroy<contact_manifold>().connect<&island_coordinator::on_destroy_contact_manifold>(*this);
}

island_coordinator::~island_coordinator() {
    for (auto &pair : m_island_ctx_map) {
        auto &ctx = pair.second;
        ctx->terminate();
    }
}

void island_coordinator::on_construct_graph_node(entt::registry &registry, entt::entity entity) {
    if (m_importing) return;

    m_new_graph_nodes.push_back(entity);

    if (registry.any_of<procedural_tag>(entity)) {
        registry.emplace<island_resident>(entity);
    } else {
        registry.emplace<multi_island_resident>(entity);
    }
}

void island_coordinator::on_construct_graph_edge(entt::registry &registry, entt::entity entity) {
    if (m_importing) return;

    m_new_graph_edges.push_back(entity);
    // Assuming this graph edge is a constraint or contact manifold, which
    // are always procedural, thus can only reside in one island.
    registry.emplace<island_resident>(entity);
}

void island_coordinator::on_destroy_graph_node(entt::registry &registry, entt::entity entity) {
    auto &node = registry.get<graph_node>(entity);
    auto &graph = registry.ctx<entity_graph>();

    // Prevent edges from being removed in `on_destroy_graph_edge`. The more
    // direct `entity_graph::remove_all_edges` will be used instead.
    registry.on_destroy<graph_edge>().disconnect<&island_coordinator::on_destroy_graph_edge>(*this);

    graph.visit_edges(node.node_index, [&] (auto edge_index) {
        auto edge_entity = graph.edge_entity(edge_index);
        registry.destroy(edge_entity);
    });

    registry.on_destroy<graph_edge>().connect<&island_coordinator::on_destroy_graph_edge>(*this);

    graph.remove_all_edges(node.node_index);
    graph.remove_node(node.node_index);
}

void island_coordinator::on_destroy_graph_edge(entt::registry &registry, entt::entity entity) {
    auto &edge = registry.get<graph_edge>(entity);
    auto &graph = registry.ctx<entity_graph>();
    graph.remove_edge(edge.edge_index);
}

void island_coordinator::on_destroy_island_resident(entt::registry &registry, entt::entity entity) {
    auto &resident = registry.get<island_resident>(entity);

    // Remove from island.
    auto &island = registry.get<edyn::island>(resident.island_entity);

    if (island.nodes.contains(entity)) {
        island.nodes.erase(entity);
    } else if (island.edges.contains(entity)) {
        island.edges.erase(entity);
    }

    if (m_importing) return;

    auto &ctx = m_island_ctx_map.at(resident.island_entity);

    // When importing delta, the entity is removed from the entity map as part
    // of the import process. Otherwise, the removal has to be done here.
    if (ctx->m_entity_map.contains_other(entity)) {
        ctx->m_entity_map.erase_other(entity);
    }

    // Notify the worker of the destruction which happened in the main registry
    // first.
    ctx->m_op_builder->destroy(entity);

    // Manually call these on_destroy functions since they could be triggered
    // by the EnTT delegate after the island resident is destroyed and the island
    // resident component is needed in these on_destroy functions.
    if (registry.any_of<contact_manifold>(entity)) {
        on_destroy_contact_manifold(registry, entity);
    }
}

void island_coordinator::on_destroy_multi_island_resident(entt::registry &registry, entt::entity entity) {
    auto &resident = registry.get<multi_island_resident>(entity);

    // Remove from islands.
    for (auto island_entity : resident.island_entities) {
        auto &island = registry.get<edyn::island>(island_entity);
        island.nodes.erase(entity);

        if (!m_importing)  {
            auto &ctx = m_island_ctx_map.at(island_entity);
            ctx->m_op_builder->destroy(entity);

            if (ctx->m_entity_map.contains_other(entity)) {
                ctx->m_entity_map.erase_other(entity);
            }
        }
    }
}

void island_coordinator::on_destroy_contact_manifold(entt::registry &registry, entt::entity entity) {
    // Trigger contact destroyed events.
    auto &manifold = registry.get<contact_manifold>(entity);

    if (manifold.num_points > 0) {
        for (unsigned i = 0; i < manifold.num_points; ++i) {
            m_contact_point_destroyed_signal.publish(entity, manifold.ids[i]);
        }

        m_contact_ended_signal.publish(entity);
    }
}

static void entity_vector_erase_invalid(std::vector<entt::entity> &vec,
                                        const entt::registry &registry) {
    auto predicate = [&] (entt::entity entity) {
        return !registry.valid(entity);
    };

    vec.erase(std::remove_if(vec.begin(), vec.end(), predicate), vec.end());
}

void island_coordinator::init_new_nodes_and_edges() {
    // Entities that were created and destroyed before a call to `edyn::update`
    // are still in these collections, thus remove invalid entities first.
    entity_vector_erase_invalid(m_new_graph_nodes, *m_registry);
    entity_vector_erase_invalid(m_new_graph_edges, *m_registry);

    if (m_new_graph_nodes.empty() && m_new_graph_edges.empty()) return;

    auto &graph = m_registry->ctx<entity_graph>();
    auto node_view = m_registry->view<graph_node>();
    auto edge_view = m_registry->view<graph_edge>();
    std::set<entity_graph::index_type> procedural_node_indices;

    for (auto entity : m_new_graph_nodes) {
        if (m_registry->any_of<procedural_tag>(entity)) {
            auto &node = node_view.get<graph_node>(entity);
            procedural_node_indices.insert(node.node_index);
        } else {
            init_new_non_procedural_node(entity);
        }
    }

    for (auto edge_entity : m_new_graph_edges) {
        auto &edge = edge_view.get<graph_edge>(edge_entity);
        auto node_entities = graph.edge_node_entities(edge.edge_index);

        if (m_registry->any_of<procedural_tag>(node_entities.first)) {
            auto &node = node_view.get<graph_node>(node_entities.first);
            procedural_node_indices.insert(node.node_index);
        }

        if (m_registry->any_of<procedural_tag>(node_entities.second)) {
            auto &node = node_view.get<graph_node>(node_entities.second);
            procedural_node_indices.insert(node.node_index);
        }
    }

    m_new_graph_nodes.clear();
    m_new_graph_edges.clear();

    if (procedural_node_indices.empty()) return;

    std::vector<entt::entity> connected_nodes;
    std::vector<entt::entity> connected_edges;
    std::vector<entt::entity> island_entities;
    auto resident_view = m_registry->view<island_resident>();
    auto procedural_view = m_registry->view<procedural_tag>();

    graph.reach(
        procedural_node_indices.begin(), procedural_node_indices.end(),
        [&] (entt::entity entity) { // visitNodeFunc
            // Always add non-procedurals to the connected component.
            // Only add procedural if it's not in an island yet.
            auto is_procedural = procedural_view.contains(entity);

            if (!is_procedural ||
                (is_procedural && resident_view.get<island_resident>(entity).island_entity == entt::null)) {
                connected_nodes.push_back(entity);
            }
        },
        [&] (entt::entity entity) { // visitEdgeFunc
            auto &edge_resident = resident_view.get<island_resident>(entity);

            if (edge_resident.island_entity == entt::null) {
                connected_edges.push_back(entity);
            } else {
                auto contains_island = vector_contains(island_entities, edge_resident.island_entity);

                if (!contains_island) {
                    island_entities.push_back(edge_resident.island_entity);
                }
            }
        },
        [&] (entity_graph::index_type node_index) { // shouldVisitFunc
            auto other_entity = graph.node_entity(node_index);

            // Collect islands involved in this connected component.
            // Always visit the non-procedural nodes. Their edges won't be
            // visited later because in the graph they're non-connecting nodes.
            if (!procedural_view.contains(other_entity)) {
                return true;
            }

            // Visit neighbor node if it's not in an island yet.
            auto &other_resident = resident_view.get<island_resident>(other_entity);

            if (other_resident.island_entity == entt::null) {
                return true;
            }

            auto contains_island = vector_contains(island_entities, other_resident.island_entity);

            if (!contains_island) {
                island_entities.push_back(other_resident.island_entity);
            }

            bool continue_visiting = false;

            // Visit neighbor if it contains an edge that is not in an island yet.
            graph.visit_edges(node_index, [&] (auto edge_index) {
                auto edge_entity = graph.edge_entity(edge_index);
                if (resident_view.get<island_resident>(edge_entity).island_entity == entt::null) {
                    continue_visiting = true;
                }
            });

            return continue_visiting;
        },
        [&] () { // connectedComponentFunc
            if (island_entities.empty()) {
                create_island(m_timestamp, false, connected_nodes, connected_edges);
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

void island_coordinator::init_new_non_procedural_node(entt::entity node_entity) {
    EDYN_ASSERT(!(m_registry->any_of<procedural_tag>(node_entity)));

    auto procedural_view = m_registry->view<procedural_tag>();
    auto resident_view = m_registry->view<island_resident>();
    auto &node = m_registry->get<graph_node>(node_entity);
    auto &resident = m_registry->get<multi_island_resident>(node_entity);

    // Add new non-procedural entity to islands of neighboring procedural entities.
    m_registry->ctx<entity_graph>().visit_neighbors(node.node_index, [&] (entt::entity other) {
        if (!procedural_view.contains(other)) return;

        auto &other_resident = resident_view.get<island_resident>(other);
        if (other_resident.island_entity == entt::null) return;

        auto &island = m_registry->get<edyn::island>(other_resident.island_entity);
        island.nodes.emplace(node_entity);

        if (!resident.island_entities.contains(other_resident.island_entity)) {
            resident.island_entities.emplace(other_resident.island_entity);
            auto &ctx = m_island_ctx_map.at(other_resident.island_entity);
            ctx->m_op_builder->create(node_entity);
            ctx->m_op_builder->emplace_all(*m_registry, node_entity);
        }
    });
}

entt::entity island_coordinator::create_island(double timestamp, bool sleeping,
                                               const std::vector<entt::entity> &nodes,
                                               const std::vector<entt::entity> &edges) {
    auto island_entity = m_registry->create();
    m_registry->emplace<island>(island_entity);
    auto &isle_time = m_registry->emplace<island_timestamp>(island_entity);
    isle_time.value = timestamp;

    auto [main_queue_input, main_queue_output] = make_message_queue_input_output();
    auto [isle_queue_input, isle_queue_output] = make_message_queue_input_output();

    // The `island_worker` is dynamically allocated and kept alive while
    // the associated island lives. The job that's created for it calls its
    // `update` function which reschedules itself to be run over and over again.
    // After the `finish` function is called on it (when the island is destroyed),
    // it will be deallocated on the next run.
    auto &settings = m_registry->ctx<edyn::settings>();
    auto &material_table = m_registry->ctx<edyn::material_mix_table>();
    auto *worker = new island_worker(island_entity, settings, material_table,
                                     message_queue_in_out(main_queue_input, isle_queue_output));

    m_island_ctx_map[island_entity] = std::make_unique<island_worker_context>(
        island_entity, worker, (*settings.make_reg_op_builder)(),
        message_queue_in_out(isle_queue_input, main_queue_output));

    auto &ctx = m_island_ctx_map[island_entity];

    // Insert the first entity mapping between the remote island entity and
    // the local island entity.
    ctx->m_entity_map.insert(worker->island_entity(), island_entity);

    // Register to receive registry operations.
    ctx->reg_op_sink().connect<&island_coordinator::on_island_reg_ops>(*this);
    ctx->split_island_sink().connect<&island_coordinator::on_split_island>(*this);

    // Send over an op containing this island entity to the island worker
    // before it even starts.
    ctx->m_op_builder->emplace<island_timestamp>(*m_registry, island_entity);

    if (sleeping) {
        m_registry->emplace<sleeping_tag>(island_entity);
        ctx->m_op_builder->emplace<sleeping_tag>(*m_registry, island_entity);
    }

    insert_to_island(*ctx, nodes, edges);

    ctx->send<msg::island_reg_ops>(ctx->m_op_builder->finish());

    // Create tree_view for this island using the procedural node AABBs. This
    // ensures expectations will be met after this function call, or else this
    // island would not have a tree_view until the worker sends an update.
    dynamic_tree tree;
    auto aabb_view = m_registry->view<AABB, procedural_tag>();

    for (auto entity : nodes) {
        if (aabb_view.contains(entity)) {
            auto &aabb = aabb_view.get<AABB>(entity);
            tree.create(aabb, entity);
        }
    }

    m_registry->emplace<tree_view>(island_entity, tree.view());

    return island_entity;
}

void island_coordinator::insert_to_island(entt::entity island_entity,
                                          const std::vector<entt::entity> &nodes,
                                          const std::vector<entt::entity> &edges) {
    auto &ctx = m_island_ctx_map.at(island_entity);
    insert_to_island(*ctx, nodes, edges);
}

void island_coordinator::insert_to_island(island_worker_context &ctx,
                                          const std::vector<entt::entity> &nodes,
                                          const std::vector<entt::entity> &edges) {

    auto &island = m_registry->get<edyn::island>(ctx.island_entity());

    for (auto entity : nodes) {
        if (!island.nodes.contains(entity)) {
            island.nodes.emplace(entity);
        }
    }

    for (auto entity : edges) {
        if (!island.edges.contains(entity)) {
            island.edges.emplace(entity);
        }
    }

    auto resident_view = m_registry->view<island_resident>();
    auto multi_resident_view = m_registry->view<multi_island_resident>();
    auto procedural_view = m_registry->view<procedural_tag>();
    auto island_entity = ctx.island_entity();

    for (auto entity : nodes) {
        if (procedural_view.contains(entity)) {
            auto &resident = resident_view.get<island_resident>(entity);
            resident.island_entity = island_entity;
            ctx.m_op_builder->create(entity);
            ctx.m_op_builder->emplace_all(*m_registry, entity);
        } else {
            auto &resident = multi_resident_view.get<multi_island_resident>(entity);

            if (resident.island_entities.contains(island_entity) == 0) {
                // Non-procedural entity is not yet in this island, thus create it.
                resident.island_entities.emplace(island_entity);
                ctx.m_op_builder->create(entity);
                ctx.m_op_builder->emplace_all(*m_registry, entity);
            }
        }
    }

    for (auto entity : edges) {
        // Assign island to residents. All edges are procedural, thus having an
        // `island_resident`, which refers to a single island.
        auto &resident = resident_view.get<island_resident>(entity);
        resident.island_entity = island_entity;
    }

    ctx.m_op_builder->create(edges.begin(), edges.end());
    ctx.m_op_builder->emplace_all(*m_registry, edges);
}

entt::entity island_coordinator::merge_islands(const std::vector<entt::entity> &island_entities,
                                               const std::vector<entt::entity> &new_nodes,
                                               const std::vector<entt::entity> &new_edges) {
    EDYN_ASSERT(island_entities.size() > 1);

    // Pick biggest island and move the other entities into it.
    entt::entity island_entity;
    size_t biggest_size = 0;

    for (auto entity : island_entities) {
        auto &island = m_registry->get<edyn::island>(entity);
        auto size = island.nodes.size() + island.edges.size();

        if (size > biggest_size) {
            biggest_size = size;
            island_entity = entity;
        }
    }

    auto other_island_entities = island_entities;
    vector_erase(other_island_entities, island_entity);

    auto all_nodes = new_nodes;
    auto all_edges = new_edges;

    for (auto other_island_entity : other_island_entities) {
        auto &island = m_registry->get<edyn::island>(other_island_entity);
        all_nodes.insert(all_nodes.end(), island.nodes.begin(), island.nodes.end());
        all_edges.insert(all_edges.end(), island.edges.begin(), island.edges.end());
    }

    auto multi_resident_view = m_registry->view<multi_island_resident>();

    for (auto entity : all_nodes) {
        // Entity might be coming from a sleeping island. Remove `sleeping_tag`s
        // since the island is supposed to be awake after a merge.
        m_registry->remove<sleeping_tag>(entity);

        // Remove islands to be destroyed from multi island residents.
        if (multi_resident_view.contains(entity)) {
            auto &resident = multi_resident_view.get<multi_island_resident>(entity);

            for (auto other_island_entity : other_island_entities) {
                if (resident.island_entities.contains(other_island_entity)) {
                    resident.island_entities.erase(other_island_entity);
                }
            }
        }
    }

    for (auto entity : all_edges) {
        m_registry->remove<sleeping_tag>(entity);
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
    if (m_registry->any_of<sleeping_tag>(island_entity)) {
        auto &isle_timestamp = m_registry->get<island_timestamp>(island_entity);
        isle_timestamp.value = m_timestamp;
    }

    return island_entity;
}

void island_coordinator::create_island(std::vector<entt::entity> nodes, bool sleeping) {
#if EDYN_DEBUG && !EDYN_DISABLE_ASSERT
    for (auto entity : nodes) {
        EDYN_ASSERT(m_registry->any_of<graph_node>(entity));
    }
#endif

    auto timestamp = performance_time();
    create_island(timestamp, sleeping, nodes, {});
}

void island_coordinator::refresh_dirty_entities() {
    auto dirty_view = m_registry->view<dirty>();
    auto resident_view = m_registry->view<island_resident>();
    auto multi_resident_view = m_registry->view<multi_island_resident>();
    auto &index_source = m_registry->ctx<settings>().index_source;

    // Do not share components which are not present in the shared components
    // list.
    auto remove_unshared = [index_source] (dirty::id_set_t &set) {
        for (auto it = set.begin(); it != set.end();) {
            if (index_source->index_of_id(*it) == SIZE_MAX) {
                it = set.erase(it);
            } else {
                ++it;
            }
        }
    };

    auto refresh = [this] (entt::entity entity, dirty &dirty, entt::entity island_entity) {
        if (!m_island_ctx_map.count(island_entity)) {
            return;
        }

        auto &ctx = m_island_ctx_map.at(island_entity);
        auto &builder = ctx->m_op_builder;

        if (dirty.is_new_entity) {
            builder->create(entity);
        }

        builder->emplace_type_ids(*m_registry, entity,
            dirty.created_indexes.begin(), dirty.created_indexes.end());
        builder->replace_type_ids(*m_registry, entity,
            dirty.updated_indexes.begin(), dirty.updated_indexes.end());
        builder->remove_type_ids(*m_registry, entity,
            dirty.destroyed_indexes.begin(), dirty.destroyed_indexes.end());
    };

    dirty_view.each([&] (entt::entity entity, dirty &dirty) {
        remove_unshared(dirty.created_indexes);
        remove_unshared(dirty.updated_indexes);
        remove_unshared(dirty.destroyed_indexes);

        if (resident_view.contains(entity)) {
            refresh(entity, dirty, resident_view.get<island_resident>(entity).island_entity);
        } else if (multi_resident_view.contains(entity)) {
            auto &resident = multi_resident_view.get<multi_island_resident>(entity);
            for (auto island_entity : resident.island_entities) {
                refresh(entity, dirty, island_entity);
            }
        }
    });

    m_registry->clear<dirty>();
}

void island_coordinator::on_island_reg_ops(entt::entity source_island_entity, const msg::island_reg_ops &msg) {
    m_importing = true;
    auto &source_ctx = m_island_ctx_map.at(source_island_entity);

    msg.ops.execute(*m_registry, source_ctx->m_entity_map);

    // Insert entity mappings for new entities into the current op.
    msg.ops.create_for_each([&] (entt::entity remote_entity) {
        auto local_entity = source_ctx->m_entity_map.at(remote_entity);
        source_ctx->m_op_builder->add_entity_mapping(local_entity, remote_entity);
    });

    auto procedural_view = m_registry->view<procedural_tag>();
    auto node_view = m_registry->view<graph_node>();
    auto &island = m_registry->get<edyn::island>(source_island_entity);

    // Insert nodes in the graph for each rigid body.
    auto &graph = m_registry->ctx<entity_graph>();
    auto insert_node = [&] (entt::entity remote_entity) {
        auto local_entity = source_ctx->m_entity_map.at(remote_entity);
        auto non_connecting = !m_registry->any_of<procedural_tag>(local_entity);
        auto node_index = graph.insert_node(local_entity, non_connecting);
        m_registry->emplace<graph_node>(local_entity, node_index);

        if (procedural_view.contains(local_entity)) {
            m_registry->emplace<island_resident>(local_entity, source_island_entity);
        } else {
            auto &resident = m_registry->emplace<multi_island_resident>(local_entity);
            resident.island_entities.emplace(source_island_entity);
        }

        island.nodes.emplace(local_entity);
    };

    msg.ops.emplace_for_each<rigidbody_tag, external_tag>(insert_node);

    // Insert edges in the graph for constraints.
    msg.ops.emplace_for_each(constraints_tuple, [&] (entt::entity remote_entity, const auto &con) {
        auto local_entity = source_ctx->m_entity_map.at(remote_entity);

        if (m_registry->any_of<graph_edge>(local_entity)) return;

        auto &node0 = node_view.get<graph_node>(source_ctx->m_entity_map.at(con.body[0]));
        auto &node1 = node_view.get<graph_node>(source_ctx->m_entity_map.at(con.body[1]));
        auto edge_index = graph.insert_edge(local_entity, node0.node_index, node1.node_index);
        m_registry->emplace<graph_edge>(local_entity, edge_index);
        m_registry->emplace<island_resident>(local_entity, source_island_entity);
        island.edges.emplace(local_entity);
    });

    m_importing = false;

    // Generate contact events.
    msg.ops.replace_for_each<contact_manifold_events>([&] (entt::entity remote_entity,
                                                           const contact_manifold_events &events) {
        auto manifold_entity = source_ctx->m_entity_map.at(remote_entity);

        if (events.contact_started) {
            m_contact_started_signal.publish(manifold_entity);
        }

        for (unsigned i = 0; i < events.num_contacts_created; ++i) {
            m_contact_point_created_signal.publish(manifold_entity, events.contacts_created[i]);
        }

        for (unsigned i = 0; i < events.num_contacts_destroyed; ++i) {
            m_contact_point_destroyed_signal.publish(manifold_entity, events.contacts_destroyed[i]);
        }

        if (events.contact_ended) {
            m_contact_ended_signal.publish(manifold_entity);
        }
    });
}

void island_coordinator::on_split_island(entt::entity source_island_entity, const msg::split_island &) {
    m_islands_to_split.push_back(source_island_entity);
}

void island_coordinator::split_islands() {
    for (auto island_entity : m_islands_to_split) {
        split_island(island_entity);
    }
    m_islands_to_split.clear();
}

void island_coordinator::split_island(entt::entity split_island_entity) {
    if (m_island_ctx_map.count(split_island_entity) == 0) return;

    auto &ctx = m_island_ctx_map.at(split_island_entity);
    auto connected_components = ctx->split();

    if (connected_components.size() <= 1) return;

    // Process any new messages enqueued during the split, such as created
    // entities that need to have their entity mappings added and the
    // update AABB `tree_view` of this island, which removes entities that
    // have moved due to the split.
    ctx->read_messages();

    // Map entities to the coordinator space.
    for (auto &connected_component : connected_components) {
        for (auto &entity : connected_component.nodes) {
            entity = ctx->m_entity_map.at(entity);
        }

        for (auto &entity : connected_component.edges) {
            entity = ctx->m_entity_map.at(entity);
        }
    }

    auto timestamp = m_registry->get<island_timestamp>(split_island_entity).value;
    bool sleeping = m_registry->any_of<sleeping_tag>(split_island_entity);
    auto multi_resident_view = m_registry->view<multi_island_resident>();
    auto procedural_view = m_registry->view<procedural_tag>();

    // Collect non-procedural entities that are still in the island that was split.
    // The first connected component in the array is the one left in the island
    // that was split.
    auto &source_connected_component = connected_components.front();
    std::vector<entt::entity> remaining_non_procedural_entities;

    for (auto entity : source_connected_component.nodes) {
        if (!procedural_view.contains(entity)) {
            remaining_non_procedural_entities.push_back(entity);
        }
    }

    auto &island = m_registry->get<edyn::island>(split_island_entity);

    for (size_t i = 1; i < connected_components.size(); ++i) {
        auto &connected = connected_components[i];
        bool contains_procedural = false;

        for (auto entity : connected.nodes) {
            if (procedural_view.contains(entity)) {
                contains_procedural = true;
                island.nodes.erase(entity);
                ctx->m_entity_map.erase_other(entity);
            } else if (!vector_contains(remaining_non_procedural_entities, entity)) {
                // Remove island that was split from multi-residents if they're not
                // present in the source island. Non-procedural could be in many
                // islands thus it's necessary to check before erasing.
                auto &resident = multi_resident_view.get<multi_island_resident>(entity);

                if (resident.island_entities.contains(split_island_entity)) {
                    resident.island_entities.erase(split_island_entity);
                }

                if (island.nodes.contains(entity)) {
                    island.nodes.erase(entity);
                    ctx->m_entity_map.erase_other(entity);
                }
            }
        }

        for (auto entity : connected.edges) {
            island.edges.erase(entity);
            ctx->m_entity_map.erase_other(entity);
        }

        // Do not create a new island if this connected component does not
        // contain any procedural node.
        if (!contains_procedural) continue;

        create_island(timestamp, sleeping, connected.nodes, connected.edges);
    }
}

void island_coordinator::sync() {
    for (auto &pair : m_island_ctx_map) {
        auto island_entity = pair.first;
        auto &ctx = pair.second;

        if (!ctx->reg_ops_empty()) {
            ctx->send_reg_ops();

            if (m_registry->any_of<sleeping_tag>(island_entity)) {
                ctx->send<msg::wake_up_island>();
            }
        }

        ctx->flush();
    }
}

void island_coordinator::update() {
    m_timestamp = performance_time();

    for (auto &pair : m_island_ctx_map) {
        pair.second->read_messages();
    }

    init_new_nodes_and_edges();
    refresh_dirty_entities();
    sync();
    split_islands();
}

void island_coordinator::set_paused(bool paused) {
    for (auto &pair : m_island_ctx_map) {
        auto &ctx = pair.second;
        ctx->send<msg::set_paused>(paused);
    }
}

void island_coordinator::step_simulation() {
    for (auto &pair : m_island_ctx_map) {
        auto island_entity = pair.first;
        if (m_registry->any_of<sleeping_tag>(island_entity)) continue;

        auto &ctx = pair.second;
        ctx->send<msg::step_simulation>();
    }
}

void island_coordinator::settings_changed() {
    auto &settings = m_registry->ctx<edyn::settings>();

    for (auto &pair : m_island_ctx_map) {
        auto &ctx = pair.second;
        ctx->send<msg::set_settings>(settings);
    }
}

void island_coordinator::material_table_changed() {
    auto &material_table = m_registry->ctx<material_mix_table>();

    for (auto &pair : m_island_ctx_map) {
        auto &ctx = pair.second;
        ctx->send<msg::set_material_table>(material_table);
    }
}

void island_coordinator::set_center_of_mass(entt::entity entity, const vector3 &com) {
    auto &resident = m_registry->get<island_resident>(entity);
    auto &ctx = m_island_ctx_map.at(resident.island_entity);
    ctx->send<msg::set_com>(entity, com);
}

}
