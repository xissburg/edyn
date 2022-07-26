#include "edyn/parallel/island_coordinator.hpp"
#include "edyn/collision/broadphase_main.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/collision/contact_manifold_events.hpp"
#include "edyn/collision/contact_point.hpp"
#include "edyn/collision/dynamic_tree.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/comp/shape_index.hpp"
#include "edyn/networking/networking_external.hpp"
#include "edyn/parallel/message.hpp"
#include "edyn/parallel/component_index_source.hpp"
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
#include <entt/entity/entity.hpp>
#include <entt/entity/registry.hpp>
#include <set>
#include <thread>

namespace edyn {

island_coordinator::island_coordinator(entt::registry &registry,unsigned short num_island_workers)
    : m_registry(&registry)
    , m_message_queue_handle(
        message_dispatcher::global().make_queue<
            msg::step_update,
            msg::entities_received_by_worker,
            msg::entities_moved,
            msg::raycast_response
        >("coordinator"))
{
    registry.on_construct<graph_node>().connect<&island_coordinator::on_construct_graph_node>(*this);
    registry.on_destroy<graph_node>().connect<&island_coordinator::on_destroy_graph_node>(*this);
    registry.on_construct<graph_edge>().connect<&island_coordinator::on_construct_graph_edge>(*this);
    registry.on_destroy<graph_edge>().connect<&island_coordinator::on_destroy_graph_edge>(*this);

    registry.on_destroy<island_worker_resident>().connect<&island_coordinator::on_destroy_island_worker_resident>(*this);
    registry.on_destroy<multi_island_worker_resident>().connect<&island_coordinator::on_destroy_multi_island_worker_resident>(*this);
    registry.on_destroy<island_tag>().connect<&island_coordinator::on_destroy_island>(*this);

    registry.on_destroy<contact_manifold>().connect<&island_coordinator::on_destroy_contact_manifold>(*this);

    m_message_queue_handle.sink<msg::step_update>().connect<&island_coordinator::on_step_update>(*this);
    m_message_queue_handle.sink<msg::entities_received_by_worker>().connect<&island_coordinator::on_entities_received>(*this);
    m_message_queue_handle.sink<msg::entities_moved>().connect<&island_coordinator::on_entities_moved>(*this);
    m_message_queue_handle.sink<msg::raycast_response>().connect<&island_coordinator::on_raycast_response>(*this);

    if (num_island_workers == 0) {
        num_island_workers = std::thread::hardware_concurrency();
    }

    for (auto i = num_island_workers; i > 0; --i) {
        create_worker();
    }
}

island_coordinator::~island_coordinator() {
    for (auto &ctx : m_worker_ctx) {
        ctx->terminate();
    }
}

void island_coordinator::on_construct_graph_node(entt::registry &registry, entt::entity entity) {
    if (m_importing) return;

    m_new_graph_nodes.push_back(entity);

    if (registry.any_of<procedural_tag>(entity)) {
        registry.emplace<island_worker_resident>(entity);
    } else {
        registry.emplace<multi_island_worker_resident>(entity);
    }
}

void island_coordinator::on_construct_graph_edge(entt::registry &registry, entt::entity entity) {
    if (m_importing) return;

    m_new_graph_edges.push_back(entity);
    registry.emplace<island_worker_resident>(entity);
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

    // Remove from worker context.
    auto &ctx = m_worker_ctx[resident.worker_index];

    if (ctx->m_nodes.contains(entity)) {
        ctx->m_nodes.erase(entity);
    } else if (ctx->m_edges.contains(entity)) {
        ctx->m_edges.erase(entity);
    } else if (registry.any_of<island_tag>(entity)) {
        ctx->m_islands.erase(entity);
    }

    if (m_importing) return;

    // When importing delta, the entity is removed from the entity map as part
    // of the import process. Otherwise, the removal has to be done here.
    if (ctx->m_entity_map.contains_local(entity)) {
        ctx->m_entity_map.erase_local(entity);
    }

    // Notify the worker of the destruction which happened in the main registry
    // first.
    ctx->m_op_builder->destroy(entity);
}

void island_coordinator::on_destroy_multi_island_worker_resident(entt::registry &registry, entt::entity entity) {
    auto &resident = registry.get<multi_island_worker_resident>(entity);

    // Remove from islands.
    for (auto idx : resident.worker_indices) {
        auto &ctx = m_worker_ctx[idx];
        ctx->m_nodes.remove(entity);

        if (!m_importing)  {
            ctx->m_op_builder->destroy(entity);

            if (ctx->m_entity_map.contains_local(entity)) {
                ctx->m_entity_map.erase_local(entity);
            }
        }
    }
}

void island_coordinator::on_destroy_island(entt::registry &registry, entt::entity entity) {
    if (auto *resident = registry.try_get<island_worker_resident>(entity)) {
        auto &ctx = m_worker_ctx[resident->worker_index];
        ctx->m_islands.erase(entity);
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
    auto procedural_view = m_registry->view<procedural_tag>();
    std::set<entity_graph::index_type> procedural_node_indices;

    for (auto entity : m_new_graph_nodes) {
        if (procedural_view.contains(entity)) {
            auto [node] = node_view.get(entity);
            procedural_node_indices.insert(node.node_index);
        } else {
            init_new_non_procedural_node(entity);
        }
    }

    for (auto edge_entity : m_new_graph_edges) {
        auto [edge] = edge_view.get(edge_entity);
        auto node_entities = graph.edge_node_entities(edge.edge_index);

        if (procedural_view.contains(node_entities.first)) {
            auto [node] = node_view.get(node_entities.first);
            procedural_node_indices.insert(node.node_index);
        }

        if (procedural_view.contains(node_entities.second)) {
            auto [node] = node_view.get(node_entities.second);
            procedural_node_indices.insert(node.node_index);
        }
    }

    m_new_graph_nodes.clear();
    m_new_graph_edges.clear();

    if (procedural_node_indices.empty()) return;

    std::vector<entt::entity> connected_nodes;
    std::vector<entt::entity> connected_edges;
    std::vector<island_worker_index_type> worker_indices;
    auto resident_view = m_registry->view<island_worker_resident>();

    graph.reach(
        procedural_node_indices.begin(), procedural_node_indices.end(),
        [&](entt::entity entity) { // visit_node_func
            // Always add non-procedurals to the connected component.
            // Only add procedural if it's not assigned to a worker yet.
            auto is_procedural = procedural_view.contains(entity);

            if (!is_procedural ||
                (is_procedural && std::get<0>(resident_view.get(entity)).worker_index == invalid_worker_index)) {
                connected_nodes.push_back(entity);
            }
        },
        [&](entt::entity entity) { // visit_edge_func
            auto [resident] = resident_view.get(entity);

            if (resident.worker_index != invalid_worker_index) {
                if (!vector_contains(worker_indices, resident.worker_index)) {
                    worker_indices.push_back(resident.worker_index);
                }
            } else {
                connected_edges.push_back(entity);
            }
        },
        [&](entity_graph::index_type node_index) { // should_visit_func
            auto other_entity = graph.node_entity(node_index);

            // Collect islands involved in this connected component.
            // Always visit the non-procedural nodes. Their edges won't be
            // visited later because in the graph they're non-connecting nodes.
            if (!procedural_view.contains(other_entity)) {
                return true;
            }

            // Visit neighbor node if it's not in a worker yet.
            auto [other_resident] = resident_view.get(other_entity);

            if (other_resident.worker_index == invalid_worker_index) {
                return true;
            }

            auto contains_worker = vector_contains(worker_indices, other_resident.worker_index);

            if (!contains_worker) {
                worker_indices.push_back(other_resident.worker_index);
            }

            bool continue_visiting = false;

            // Visit neighbor if it contains an edge that is not in a worker yet.
            graph.visit_edges(node_index, [&](auto edge_index) {
                auto edge_entity = graph.edge_entity(edge_index);
                if (std::get<0>(resident_view.get(edge_entity)).worker_index == invalid_worker_index) {
                    continue_visiting = true;
                }
            });

            return continue_visiting;
        },
        [&]() { // connected_component_func
            if (worker_indices.empty()) {
                // Insert into any worker.
                batch_nodes(connected_nodes, connected_edges);
            } else if (worker_indices.size() == 1) {
                insert_to_worker(worker_indices[0], connected_nodes, connected_edges);
            } else {
                // TODO: move islands into a single worker and then create nodes and edges
                // after acknowledgement.
                EDYN_ASSERT(false);
                //merge_islands(island_entities, connected_nodes, connected_edges);
            }

            connected_nodes.clear();
            connected_edges.clear();
            worker_indices.clear();
        });
}

void island_coordinator::init_new_non_procedural_node(entt::entity node_entity) {
    EDYN_ASSERT(!(m_registry->any_of<procedural_tag>(node_entity)));

    auto procedural_view = m_registry->view<procedural_tag>();
    auto resident_view = m_registry->view<island_worker_resident>();
    auto &node = m_registry->get<graph_node>(node_entity);
    auto &resident = m_registry->get<multi_island_worker_resident>(node_entity);

    // Add new non-procedural entity to workers where neighboring procedural
    // entities currently reside.
    m_registry->ctx().at<entity_graph>().visit_neighbors(node.node_index, [&](entt::entity other) {
        if (!procedural_view.contains(other)) return;

        auto [other_resident] = resident_view.get(other);

        if (other_resident.worker_index == invalid_worker_index) return;

        if (!resident.worker_indices.count(other_resident.worker_index)) {
            resident.worker_indices.insert(other_resident.worker_index);
        }
    });

    for (auto idx : resident.worker_indices) {
        auto &ctx = m_worker_ctx[idx];

        if (!ctx->m_nodes.contains(node_entity)) {
            ctx->m_nodes.emplace(node_entity);
            ctx->m_op_builder->create(node_entity);
            ctx->m_op_builder->emplace_all(*m_registry, node_entity);
        }
    }
}

island_worker_index_type island_coordinator::create_worker() {
    // The `island_worker` is dynamically allocated and kept alive while
    // the associated island lives. The job that's created for it calls its
    // `update` function which reschedules itself to be run over and over again.
    // After the `finish` function is called on it (when the island is destroyed),
    // it will be deallocated on the next run.
    auto worker_index = m_worker_ctx.size();
    auto name = "worker-" + std::to_string(worker_index);
    auto &settings = m_registry->ctx().at<edyn::settings>();
    auto &material_table = m_registry->ctx().at<edyn::material_mix_table>();
    auto *worker = new island_worker(name, settings, material_table, m_message_queue_handle.identifier);

    auto &ctx = m_worker_ctx.emplace_back(std::make_unique<island_worker_context>(
        worker, (*settings.make_reg_op_builder)()));
    ctx->m_timestamp = performance_time();

    return worker_index;
}

void island_coordinator::batch_nodes(const std::vector<entt::entity> &nodes,
                                     const std::vector<entt::entity> &edges) {
    EDYN_ASSERT(!m_worker_ctx.empty());

    // Find least busy worker.
    auto worker_index = SIZE_MAX;
    auto smallest_size = std::numeric_limits<size_t>::max();
    auto stats_view = m_registry->view<island_stats>();

    for (size_t i = 0; i < m_worker_ctx.size(); ++i) {
        auto worker_size = size_t(0);

        for (auto entity : m_worker_ctx[i]->m_islands) {
            auto [stats] = stats_view.get(entity);
            worker_size += stats.size();
        }

        if (worker_size < smallest_size) {
            smallest_size = worker_size;
            worker_index = i;
        }
    }

    insert_to_worker(worker_index, nodes, edges);
}

void island_coordinator::insert_to_worker(island_worker_index_type worker_index,
                                          const std::vector<entt::entity> &nodes,
                                          const std::vector<entt::entity> &edges) {
    EDYN_ASSERT(worker_index < m_worker_ctx.size());
    auto &ctx = *m_worker_ctx[worker_index];
    ctx.m_nodes.insert(nodes.begin(), nodes.end());
    ctx.m_edges.insert(edges.begin(), edges.end());

    auto resident_view = m_registry->view<island_worker_resident>();
    auto multi_resident_view = m_registry->view<multi_island_worker_resident>();
    auto procedural_view = m_registry->view<procedural_tag>();

    for (auto entity : nodes) {
        if (procedural_view.contains(entity)) {
            auto [resident] = resident_view.get(entity);
            resident.worker_index = worker_index;
            ctx.m_op_builder->create(entity);
            ctx.m_op_builder->emplace_all(*m_registry, entity);
        } else {
            auto [resident] = multi_resident_view.get(entity);

            if (resident.worker_indices.count(worker_index) == 0) {
                // Non-procedural entity is not yet in this island, thus create it.
                resident.worker_indices.emplace(worker_index);
                ctx.m_op_builder->create(entity);
                ctx.m_op_builder->emplace_all(*m_registry, entity);
            }
        }
    }

    for (auto entity : edges) {
        // Assign island to residents. All edges are procedural, thus having an
        // `island_worker_resident`, which refers to a single worker.
        auto [resident] = resident_view.get(entity);
        resident.worker_index = worker_index;
    }

    ctx.m_op_builder->create(edges.begin(), edges.end());
    ctx.m_op_builder->emplace_all(*m_registry, edges);
}

double island_coordinator::get_worker_timestamp(island_worker_index_type worker_index) const {
    EDYN_ASSERT(worker_index < m_worker_ctx.size());
    return m_worker_ctx[worker_index]->m_timestamp;
}

void island_coordinator::move_non_procedural_into_worker(entt::entity np_entity, island_worker_index_type worker_index) {
    EDYN_ASSERT(!m_registry->all_of<procedural_tag>(np_entity));
    insert_to_worker(worker_index, {np_entity}, {});
}

void island_coordinator::refresh_dirty_entities() {
    auto dirty_view = m_registry->view<dirty>();
    auto resident_view = m_registry->view<island_worker_resident>();
    auto multi_resident_view = m_registry->view<multi_island_worker_resident>();
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

    auto refresh = [this](entt::entity entity, dirty &dirty, island_worker_index_type worker_index) {
        auto &ctx = m_worker_ctx[worker_index];
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
            refresh(entity, dirty, resident_view.get<island_worker_resident>(entity).worker_index);
        } else if (multi_resident_view.contains(entity)) {
            auto &resident = multi_resident_view.get<multi_island_worker_resident>(entity);
            for (auto idx : resident.worker_indices) {
                refresh(entity, dirty, idx);
            }
        }
    });

    m_registry->clear<dirty>();
}

static
island_worker_index_type get_message_source_worker_index(const message_queue_identifier &id) {
    auto &name = id.value;
    auto prefix = std::string("worker-");
    EDYN_ASSERT(name.compare(0, prefix.size(), prefix) == 0);
    auto source_worker_index = std::stoi(name.substr(prefix.size(), name.size() - prefix.size()));
    return source_worker_index;
}

void island_coordinator::on_step_update(const message<msg::step_update> &msg) {
    m_importing = true;
    auto &registry = *m_registry;

    auto source_worker_index = get_message_source_worker_index(msg.sender);
    auto &source_ctx = m_worker_ctx[source_worker_index];
    auto &ops = msg.content.ops;

    ops.execute(registry, source_ctx->m_entity_map);

    // Insert entity mappings for new entities into the current op.
    ops.create_for_each([&](entt::entity remote_entity) {
        if (source_ctx->m_entity_map.contains(remote_entity)) {
            auto local_entity = source_ctx->m_entity_map.at(remote_entity);
            source_ctx->m_op_builder->add_entity_mapping(local_entity, remote_entity);
        }
    });

    source_ctx->m_timestamp = msg.content.timestamp;

    auto procedural_view = registry.view<procedural_tag>();
    auto node_view = registry.view<graph_node>();

    // Insert nodes in the graph for each new rigid body.
    auto &graph = registry.ctx().at<entity_graph>();
    auto insert_node = [&](entt::entity remote_entity) {
        auto local_entity = source_ctx->m_entity_map.at(remote_entity);
        auto non_connecting = !registry.any_of<procedural_tag>(local_entity);
        auto node_index = graph.insert_node(local_entity, non_connecting);
        registry.emplace<graph_node>(local_entity, node_index);

        if (procedural_view.contains(local_entity)) {
            registry.emplace<island_worker_resident>(local_entity, source_worker_index);
        } else {
            auto &resident = registry.emplace<multi_island_worker_resident>(local_entity);
            resident.worker_indices.emplace(source_worker_index);
        }

        source_ctx->m_nodes.emplace(local_entity);
    };

    ops.emplace_for_each<rigidbody_tag, external_tag>(insert_node);

    // Insert edges in the graph for constraints.
    ops.emplace_for_each(constraints_tuple, [&](entt::entity remote_entity, const auto &con) {
        auto local_entity = source_ctx->m_entity_map.at(remote_entity);

        if (registry.any_of<graph_edge>(local_entity)) return;

        auto [node0] = node_view.get(source_ctx->m_entity_map.at(con.body[0]));
        auto [node1] = node_view.get(source_ctx->m_entity_map.at(con.body[1]));
        auto edge_index = graph.insert_edge(local_entity, node0.node_index, node1.node_index);
        registry.emplace<graph_edge>(local_entity, edge_index);
        registry.emplace<island_worker_resident>(local_entity, source_worker_index);
        source_ctx->m_edges.emplace(local_entity);
    });

    m_importing = false;

    // Generate contact events.
    ops.replace_for_each<contact_manifold_events>([&](entt::entity remote_entity,
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

    // Assign worker resident to islands.
    ops.emplace_for_each<island_tag>([&](entt::entity remote_entity) {
        if (!source_ctx->m_entity_map.contains(remote_entity)) return;

        auto local_entity = source_ctx->m_entity_map.at(remote_entity);
        m_registry->emplace<island_worker_resident>(local_entity, source_worker_index);
        source_ctx->m_islands.emplace(local_entity);
    });

    (*g_mark_replaced_network_dirty)(registry, ops, source_ctx->m_entity_map, m_timestamp);
}

void island_coordinator::on_entities_received(const message<msg::entities_received_by_worker> &msg) {
    auto worker_index = get_message_source_worker_index(msg.sender);
    auto &ctx = m_worker_ctx[worker_index];
    auto &emap = ctx->m_entity_map;
    auto resident_view = m_registry->view<island_worker_resident>();
    auto multi_resident_view = m_registry->view<multi_island_worker_resident>();

    for (auto remote_entity : msg.content.entities) {
        if (msg.content.emap.contains(remote_entity)) {
            auto local_entity = msg.content.emap.at(remote_entity);

            if (emap.contains(remote_entity)) {
                EDYN_ASSERT(emap.at(remote_entity) == local_entity);
            } else {
                emap.insert(remote_entity, local_entity);
            }

            if (m_registry->all_of<graph_node>(local_entity)) {
                if (!ctx->m_nodes.contains(local_entity)) {
                    ctx->m_nodes.emplace(local_entity);
                }
            } else if (m_registry->all_of<graph_edge>(local_entity)) {
                ctx->m_edges.emplace(local_entity);
            }

            if (resident_view.contains(local_entity)) {
                auto [resident] = resident_view.get(local_entity);
                resident.worker_index = worker_index;
            } else if (multi_resident_view.contains(local_entity)) {
                auto [resident] = multi_resident_view.get(local_entity);

                if (!resident.worker_indices.count(worker_index)) {
                    resident.worker_indices.insert(worker_index);
                }
            }
        } else {
            // TODO: query unknown entity and components.
            EDYN_ASSERT(false);
        }
    }
}

void island_coordinator::on_entities_moved(const message<msg::entities_moved> &msg) {
    auto worker_index = get_message_source_worker_index(msg.sender);
    auto &ctx = m_worker_ctx[worker_index];
    auto &emap = ctx->m_entity_map;
    auto resident_view = m_registry->view<island_worker_resident>();
    auto multi_resident_view = m_registry->view<multi_island_worker_resident>();

    for (auto remote_entity : msg.content.entities) {
        if (!emap.contains(remote_entity)) continue;

        auto local_entity = emap.at(remote_entity);
        emap.erase(remote_entity);

        if (ctx->m_nodes.contains(local_entity)) {
            ctx->m_nodes.erase(local_entity);
        } else if (ctx->m_edges.contains(local_entity)) {
            ctx->m_edges.erase(local_entity);
        }

        if (resident_view.contains(local_entity)) {
            auto [resident] = resident_view.get(local_entity);

            if (resident.worker_index == worker_index) {
                //resident.worker_index = invalid_worker_index;
            }
        } else if (multi_resident_view.contains(local_entity)) {
            auto [resident] = multi_resident_view.get(local_entity);
            resident.worker_indices.erase(worker_index);
        }
    }
}

void island_coordinator::on_raycast_response(const message<msg::raycast_response> &msg) {
    auto &res = msg.content;
    auto &ctx = m_raycast_ctx.at(res.id);
    EDYN_ASSERT(ctx.counter > 0);
    --ctx.counter;

    if (res.result.fraction < ctx.result.fraction) {
        ctx.result = res.result;
        auto worker_index = get_message_source_worker_index(msg.sender);
        ctx.result.entity = m_worker_ctx[worker_index]->m_entity_map.at(ctx.result.entity);
    }

    if (ctx.counter == 0) {
        ctx.delegate(ctx.result);
        m_raycast_ctx.erase(res.id);
    }
}

void island_coordinator::sync() {
    for (auto &ctx : m_worker_ctx) {
        if (!ctx->reg_ops_empty()) {
            ctx->send_reg_ops(m_message_queue_handle.identifier);
        }

        ctx->flush();
    }
}

void island_coordinator::update() {
    m_timestamp = performance_time();

    // Insert dirty components into the registry ops before importing messages,
    // which could override the state of these components that have been modified
    // by the user.
    refresh_dirty_entities();

    m_message_queue_handle.update();

    init_new_nodes_and_edges();
    sync();

    balance_workers();
}

void island_coordinator::set_paused(bool paused) {
    for (auto &ctx : m_worker_ctx) {
        ctx->send<msg::set_paused>(m_message_queue_handle.identifier, paused);
    }
}

void island_coordinator::step_simulation() {
    for (auto &ctx : m_worker_ctx) {
        ctx->send<msg::step_simulation>(m_message_queue_handle.identifier);
    }
}

void island_coordinator::settings_changed() {
    auto &settings = m_registry->ctx().at<edyn::settings>();

    for (auto &ctx : m_worker_ctx) {
        ctx->send<msg::set_settings>(m_message_queue_handle.identifier, settings);
    }
}

void island_coordinator::material_table_changed() {
    auto &material_table = m_registry->ctx().at<material_mix_table>();

    for (auto &ctx : m_worker_ctx) {
        ctx->send<msg::set_material_table>(m_message_queue_handle.identifier, material_table);
    }
}

void island_coordinator::set_center_of_mass(entt::entity entity, const vector3 &com) {
    auto &resident = m_registry->get<island_worker_resident>(entity);
    EDYN_ASSERT(resident.worker_index < m_worker_ctx.size());
    auto &ctx = m_worker_ctx[resident.worker_index];
    ctx->send<msg::set_com>(m_message_queue_handle.identifier, entity, com);
}

void island_coordinator::balance_workers() {
    // Find the biggest and smallest workers. Suggest exchanging islands
    // if they have a significant difference in size.
    auto smallest_size = std::numeric_limits<size_t>::max();
    auto biggest_size = size_t(0);
    auto smallest_idx = SIZE_MAX;
    auto biggest_idx = SIZE_MAX;
    auto stats_view = m_registry->view<island_stats>();

    for (size_t i = 0; i < m_worker_ctx.size(); ++i) {
        auto &ctx = m_worker_ctx[i];
        auto worker_size = size_t(0);

        for (auto entity : ctx->m_islands) {
            auto [stats] = stats_view.get(entity);
            worker_size += stats.size();
        }

        if (worker_size < smallest_size) {
            smallest_size = worker_size;
            smallest_idx = i;
        }

        if (worker_size > biggest_size && ctx->m_islands.size() > 1) {
            biggest_size = worker_size;
            biggest_idx = i;
        }
    }

    if (smallest_idx == SIZE_MAX || biggest_idx == SIZE_MAX || smallest_idx == biggest_idx) {
        return;
    }

    // If the size of the smallest worker is less than 70% of the biggest,
    // suggest the bigger one to send islands to the smaller to balance work.
    if (smallest_size * 100 < biggest_size * 70) {
        exchange_islands_from_to(biggest_idx, smallest_idx);
    }
}

void island_coordinator::exchange_islands(island_worker_index_type worker_indexA,
                                          island_worker_index_type worker_indexB) {
    // Ask the least busy of the two workers to send relevant islands to the
    // other.
    auto stats_view = m_registry->view<island_stats>();
    auto worker_sizeA = size_t{};
    auto worker_sizeB = size_t{};
    auto &ctxA = m_worker_ctx[worker_indexA];
    auto &ctxB = m_worker_ctx[worker_indexB];

    for (auto entity : ctxA->m_islands) {
        auto [stats] = stats_view.get(entity);
        worker_sizeA += stats.size();
    }

    for (auto entity : ctxB->m_islands) {
        auto [stats] = stats_view.get(entity);
        worker_sizeB += stats.size();
    }

    if (worker_sizeA > worker_sizeB) {
        exchange_islands_from_to(worker_indexA, worker_indexB);
    } else {
        exchange_islands_from_to(worker_indexB, worker_indexA);
    }
}

void island_coordinator::exchange_islands_from_to(island_worker_index_type from_worker,
                                                  island_worker_index_type to_worker) {
    EDYN_ASSERT(from_worker != to_worker);
    auto &from_ctx = m_worker_ctx[from_worker];
    auto &to_ctx = m_worker_ctx[to_worker];

    auto msg = msg::exchange_islands{};
    msg.destination = to_ctx->message_queue_id();

    auto aabb_view = m_registry->view<island_AABB>();

    for (auto entity : to_ctx->m_islands) {
        auto [aabb] = aabb_view.get(entity);
        msg.island_aabbs.push_back(aabb);
    }

    from_ctx->send<msg::exchange_islands>(m_message_queue_handle.identifier, std::move(msg));
}

}
