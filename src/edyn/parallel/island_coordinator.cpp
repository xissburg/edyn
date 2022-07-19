#include "edyn/parallel/island_coordinator.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/collision/contact_point.hpp"
#include "edyn/collision/dynamic_tree.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/comp/shape_index.hpp"
#include "edyn/networking/networking_external.hpp"
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

    registry.on_destroy<island_worker_resident>().connect<&island_coordinator::on_destroy_island_worker_resident>(*this);
    registry.on_destroy<multi_island_worker_resident>().connect<&island_coordinator::on_destroy_multi_island_worker_resident>(*this);

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
    auto &graph = registry.ctx().at<entity_graph>();

    // Prevent edges from being removed in `on_destroy_graph_edge`. The more
    // direct `entity_graph::remove_all_edges` will be used instead.
    registry.on_destroy<graph_edge>().disconnect<&island_coordinator::on_destroy_graph_edge>(*this);

    graph.visit_edges(node.node_index, [&](auto edge_index) {
        auto edge_entity = graph.edge_entity(edge_index);
        registry.destroy(edge_entity);
    });

    registry.on_destroy<graph_edge>().connect<&island_coordinator::on_destroy_graph_edge>(*this);

    graph.remove_all_edges(node.node_index);
    graph.remove_node(node.node_index);
}

void island_coordinator::on_destroy_graph_edge(entt::registry &registry, entt::entity entity) {
    auto &edge = registry.get<graph_edge>(entity);
    auto &graph = registry.ctx().at<entity_graph>();
    graph.remove_edge(edge.edge_index);
}

void island_coordinator::on_destroy_island_worker_resident(entt::registry &registry, entt::entity entity) {
    auto &resident = registry.get<island_worker_resident>(entity);

    // Remove from island.
    auto &ctx = m_island_ctx_map.at(resident.worker_entity);
    ctx->m_nodes.erase(entity);
    ctx->m_edges.erase(entity);

    if (m_importing) return;

    // When importing delta, the entity is removed from the entity map as part
    // of the import process. Otherwise, the removal has to be done here.
    if (ctx->m_entity_map.contains_other(entity)) {
        ctx->m_entity_map.erase_other(entity);
    }

    // Notify the worker of the destruction which happened in the main registry
    // first.
    ctx->m_op_builder->destroy(entity);
}

void island_coordinator::on_destroy_multi_island_worker_resident(entt::registry &registry, entt::entity entity) {
    auto &resident = registry.get<multi_island_worker_resident>(entity);

    // Remove from islands.
    for (auto worker_entity : resident.worker_entities) {
        auto &ctx = m_island_ctx_map.at(worker_entity);
        ctx.m_nodes.remove(entity);

        if (!m_importing)  {
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
    auto predicate = [&](entt::entity entity) {
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

    auto &graph = m_registry->ctx().at<entity_graph>();
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
    std::vector<entt::entity> worker_entities;
    auto resident_view = m_registry->view<island_resident>();
    auto procedural_view = m_registry->view<procedural_tag>();

    graph.reach(
        procedural_node_indices.begin(), procedural_node_indices.end(),
        [&](entt::entity entity) { // visitNodeFunc
            // Always add non-procedurals to the connected component.
            // Only add procedural if it's not in an island yet.
            auto is_procedural = procedural_view.contains(entity);

            if (!is_procedural ||
                (is_procedural && resident_view.get<island_resident>(entity).island_entity == entt::null)) {
                connected_nodes.push_back(entity);
            }
        },
        [&](entt::entity entity) { // visitEdgeFunc
            auto &edge_resident = resident_view.get<island_resident>(entity);

            if (edge_resident.island_entity == entt::null) {
                connected_edges.push_back(entity);
            } else {
                auto contains_island = vector_contains(worker_entities, edge_resident.island_entity);

                if (!contains_island) {
                    worker_entities.push_back(edge_resident.island_entity);
                }
            }
        },
        [&](entity_graph::index_type node_index) { // shouldVisitFunc
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

            auto contains_island = vector_contains(worker_entities, other_resident.island_entity);

            if (!contains_island) {
                worker_entities.push_back(other_resident.island_entity);
            }

            bool continue_visiting = false;

            // Visit neighbor if it contains an edge that is not in an island yet.
            graph.visit_edges(node_index, [&](auto edge_index) {
                auto edge_entity = graph.edge_entity(edge_index);
                if (resident_view.get<island_resident>(edge_entity).island_entity == entt::null) {
                    continue_visiting = true;
                }
            });

            return continue_visiting;
        },
        [&]() { // connectedComponentFunc
            if (worker_entities.empty()) {
                // TODO: select least busy worker to insert new nodes in.
                auto &worker = *m_island_ctx_map.begin()->second;
                insert_to_worker(worker, connected_nodes, connected_edges);
            } else if (worker_entities.size() == 1) {
                auto worker_entity = *worker_entities.begin();
                insert_to_worker(worker_entity, connected_nodes, connected_edges);
            } else {
                // TODO: move islands into a single worker and then create nodes and edges
                // after acknowledgement.
                //merge_islands(island_entities, connected_nodes, connected_edges);
            }

            connected_nodes.clear();
            connected_edges.clear();
            worker_entities.clear();
        });
}

void island_coordinator::init_new_non_procedural_node(entt::entity node_entity) {
    EDYN_ASSERT(!(m_registry->any_of<procedural_tag>(node_entity)));

    auto procedural_view = m_registry->view<procedural_tag>();
    auto resident_view = m_registry->view<island_resident>();
    auto &node = m_registry->get<graph_node>(node_entity);
    auto &resident = m_registry->get<multi_island_resident>(node_entity);

    // Add new non-procedural entity to islands of neighboring procedural entities.
    m_registry->ctx().at<entity_graph>().visit_neighbors(node.node_index, [&](entt::entity other) {
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

entt::entity island_coordinator::create_worker() {
    auto worker_entity = m_registry->create();
    auto &isle_time = m_registry->emplace<island_worker_timestamp>(worker_entity);
    isle_time.value = performance_time();

    auto [main_queue_input, main_queue_output] = make_message_queue_input_output();
    auto [isle_queue_input, isle_queue_output] = make_message_queue_input_output();

    // The `island_worker` is dynamically allocated and kept alive while
    // the associated island lives. The job that's created for it calls its
    // `update` function which reschedules itself to be run over and over again.
    // After the `finish` function is called on it (when the island is destroyed),
    // it will be deallocated on the next run.
    auto &settings = m_registry->ctx().at<edyn::settings>();
    auto &material_table = m_registry->ctx().at<edyn::material_mix_table>();
    auto *worker = new island_worker(settings, material_table,
                                     message_queue_in_out(main_queue_input, isle_queue_output));

    m_island_ctx_map[worker_entity] = std::make_unique<island_worker_context>(
        worker_entity, worker, (*settings.make_reg_op_builder)(),
        message_queue_in_out(isle_queue_input, main_queue_output));

    // Register to receive delta.
    auto &ctx = *m_island_ctx_map[worker_entity];
    ctx.reg_op_sink().connect<&island_coordinator::on_island_reg_ops>(*this);

    return worker_entity;
}

void island_coordinator::batch_nodes(const std::vector<entt::entity> &nodes,
                                     const std::vector<entt::entity> &edges) {
    auto worker_entity = m_island_ctx_map.begin()->first;
    auto &ctx = *m_island_ctx_map[worker_entity];
    insert_to_worker(ctx, nodes, edges);
    ctx.send<msg::island_reg_ops>(ctx.m_op_builder->finish());
}

void island_coordinator::insert_to_worker(entt::entity island_entity,
                                          const std::vector<entt::entity> &nodes,
                                          const std::vector<entt::entity> &edges) {
    auto &ctx = m_island_ctx_map.at(island_entity);
    insert_to_worker(*ctx, nodes, edges);
}

void island_coordinator::insert_to_worker(island_worker_context &ctx,
                                          const std::vector<entt::entity> &nodes,
                                          const std::vector<entt::entity> &edges) {
    ctx.m_nodes.insert(nodes.begin(), nodes.end());
    ctx.m_edges.insert(edges.begin(), edges.end());

    auto resident_view = m_registry->view<island_worker_resident>();
    auto multi_resident_view = m_registry->view<multi_island_worker_resident>();
    auto procedural_view = m_registry->view<procedural_tag>();
    auto worker_entity = ctx.worker_entity();

    for (auto entity : nodes) {
        if (procedural_view.contains(entity)) {
            auto &resident = resident_view.get<island_worker_resident>(entity);
            resident.worker_entity = worker_entity;
            ctx.m_op_builder->create(entity);
            ctx.m_op_builder->emplace_all(*m_registry, entity);
        } else {
            auto &resident = multi_resident_view.get<multi_island_worker_resident>(entity);

            if (resident.worker_entities.contains(worker_entity) == 0) {
                // Non-procedural entity is not yet in this island, thus create it.
                resident.worker_entities.emplace(worker_entity);
                ctx.m_op_builder->create(entity);
                ctx.m_op_builder->emplace_all(*m_registry, entity);
            }
        }
    }

    for (auto entity : edges) {
        // Assign island to residents. All edges are procedural, thus having an
        // `island_worker_resident`, which refers to a single worker.
        auto &resident = resident_view.get<island_worker_resident>(entity);
        resident.worker_entity = worker_entity;
    }

    ctx.m_op_builder->create(edges.begin(), edges.end());
    ctx.m_op_builder->emplace_all(*m_registry, edges);
}

void island_coordinator::refresh_dirty_entities() {
    auto dirty_view = m_registry->view<dirty>();
    auto resident_view = m_registry->view<island_resident>();
    auto multi_resident_view = m_registry->view<multi_island_resident>();
    auto &index_source = m_registry->ctx().at<settings>().index_source;

    // Do not share components which are not present in the shared components
    // list.
    auto remove_unshared = [index_source](dirty::id_set_t &set) {
        for (auto it = set.begin(); it != set.end();) {
            if (index_source->index_of_id(*it) == SIZE_MAX) {
                it = set.erase(it);
            } else {
                ++it;
            }
        }
    };

    auto refresh = [this](entt::entity entity, dirty &dirty, entt::entity island_entity) {
        if (!m_island_ctx_map.count(island_entity)) {
            return;
        }

        auto &ctx = m_island_ctx_map.at(island_entity);
        auto &builder = ctx->m_op_builder;

        if (dirty.is_new_entity) {
            builder->create(entity);
        }

        builder->emplace_type_ids(*m_registry, entity,
            dirty.created_ids.begin(), dirty.created_ids.end());
        builder->replace_type_ids(*m_registry, entity,
            dirty.updated_ids.begin(), dirty.updated_ids.end());
        builder->remove_type_ids(*m_registry, entity,
            dirty.destroyed_ids.begin(), dirty.destroyed_ids.end());
    };

    dirty_view.each([&](entt::entity entity, dirty &dirty) {
        remove_unshared(dirty.created_ids);
        remove_unshared(dirty.updated_ids);
        remove_unshared(dirty.destroyed_ids);

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

void island_coordinator::on_island_reg_ops(entt::entity source_worker_entity, const msg::island_reg_ops &msg) {
    m_importing = true;
    auto &registry = *m_registry;
    auto &source_ctx = m_island_ctx_map.at(source_worker_entity);

    msg.ops.execute(registry, source_ctx->m_entity_map);

    // Insert entity mappings for new entities into the current op.
    msg.ops.create_for_each([&](entt::entity remote_entity) {
        auto local_entity = source_ctx->m_entity_map.at(remote_entity);
        source_ctx->m_op_builder->add_entity_mapping(local_entity, remote_entity);
    });

    auto procedural_view = registry.view<procedural_tag>();
    auto node_view = registry.view<graph_node>();
    auto &island = registry.get<edyn::island>(source_worker_entity);

    // Insert nodes in the graph for each rigid body.
    auto &graph = registry.ctx().at<entity_graph>();
    auto insert_node = [&](entt::entity remote_entity) {
        auto local_entity = source_ctx->m_entity_map.at(remote_entity);
        auto non_connecting = !registry.any_of<procedural_tag>(local_entity);
        auto node_index = graph.insert_node(local_entity, non_connecting);
        registry.emplace<graph_node>(local_entity, node_index);

        if (procedural_view.contains(local_entity)) {
            registry.emplace<island_resident>(local_entity, source_worker_entity);
        } else {
            auto &resident = registry.emplace<multi_island_resident>(local_entity);
            resident.island_entities.emplace(source_worker_entity);
        }

        island.nodes.emplace(local_entity);
    };

    msg.ops.emplace_for_each<rigidbody_tag, external_tag>(insert_node);

    // Insert edges in the graph for constraints.
    msg.ops.emplace_for_each(constraints_tuple, [&](entt::entity remote_entity, const auto &con) {
        auto local_entity = source_ctx->m_entity_map.at(remote_entity);

        if (registry.any_of<graph_edge>(local_entity)) return;

        auto &node0 = node_view.get<graph_node>(source_ctx->m_entity_map.at(con.body[0]));
        auto &node1 = node_view.get<graph_node>(source_ctx->m_entity_map.at(con.body[1]));
        auto edge_index = graph.insert_edge(local_entity, node0.node_index, node1.node_index);
        registry.emplace<graph_edge>(local_entity, edge_index);
        registry.emplace<island_resident>(local_entity, source_worker_entity);
        island.edges.emplace(local_entity);
    });

    m_importing = false;

    // Generate contact events.
    msg.ops.replace_for_each<contact_manifold_events>([&](entt::entity remote_entity,
                                                          const contact_manifold_events &events) {
        if (!source_ctx->m_entity_map.contains(remote_entity)) {
            return;
        }

        auto manifold_entity = source_ctx->m_entity_map.at(remote_entity);

        // Contact could have ended and started again in the same step. Do not
        // generate event in that case.
        if (events.contact_started && !events.contact_ended) {
            m_contact_started_signal.publish(manifold_entity);
        }

        for (unsigned i = 0; i < events.num_contacts_created; ++i) {
            m_contact_point_created_signal.publish(manifold_entity, events.contacts_created[i]);
        }

        for (unsigned i = 0; i < events.num_contacts_destroyed; ++i) {
            m_contact_point_destroyed_signal.publish(manifold_entity, events.contacts_destroyed[i]);
        }

        if (events.contact_ended && !events.contact_started) {
            m_contact_ended_signal.publish(manifold_entity);
        }
    });

    msg.ops.emplace_for_each<island_AABB>([&](entt::entity remote_entity, const island_AABB &) {
        if (!source_ctx->m_entity_map.contains(remote_entity)) return;

        auto local_entity = source_ctx->m_entity_map.at(remote_entity);
        m_registry->emplace<island_worker_resident>(local_entity, source_worker_entity);
    });

    (*g_mark_replaced_network_dirty)(registry, msg.ops, source_ctx->m_entity_map, m_timestamp);
}

void island_coordinator::sync() {
    for (auto &pair : m_island_ctx_map) {
        auto &ctx = pair.second;

        if (!ctx->reg_ops_empty()) {
            ctx->send_reg_ops();
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
    auto &settings = m_registry->ctx().at<edyn::settings>();

    for (auto &pair : m_island_ctx_map) {
        auto &ctx = pair.second;
        ctx->send<msg::set_settings>(settings);
    }
}

void island_coordinator::material_table_changed() {
    auto &material_table = m_registry->ctx().at<material_mix_table>();

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
