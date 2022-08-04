#include "edyn/simulation/island_coordinator.hpp"
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
#include "edyn/replication/component_index_source.hpp"
#include "edyn/shapes/shapes.hpp"
#include "edyn/config/config.h"
#include "edyn/constraints/constraint.hpp"
#include "edyn/constraints/contact_constraint.hpp"
#include "edyn/simulation/simulation_worker.hpp"
#include "edyn/comp/dirty.hpp"
#include "edyn/time/time.hpp"
#include "edyn/core/entity_graph.hpp"
#include "edyn/comp/graph_node.hpp"
#include "edyn/comp/graph_edge.hpp"
#include "edyn/util/vector_util.hpp"
#include "edyn/replication/registry_operation.hpp"
#include "edyn/context/settings.hpp"
#include "edyn/dynamics/material_mixing.hpp"
#include <entt/entity/entity.hpp>
#include <entt/entity/registry.hpp>
#include <set>

namespace edyn {

island_coordinator::island_coordinator(entt::registry &registry)
    : m_registry(&registry)
    , m_message_queue_handle(
        message_dispatcher::global().make_queue<
            msg::step_update,
            msg::raycast_response
        >("coordinator"))
{
    registry.on_construct<graph_node>().connect<&island_coordinator::on_construct_graph_node>(*this);
    registry.on_destroy<graph_node>().connect<&island_coordinator::on_destroy_graph_node>(*this);
    registry.on_construct<graph_edge>().connect<&island_coordinator::on_construct_graph_edge>(*this);
    registry.on_destroy<graph_edge>().connect<&island_coordinator::on_destroy_graph_edge>(*this);
    registry.on_destroy<contact_manifold>().connect<&island_coordinator::on_destroy_contact_manifold>(*this);

    m_message_queue_handle.sink<msg::step_update>().connect<&island_coordinator::on_step_update>(*this);
    m_message_queue_handle.sink<msg::raycast_response>().connect<&island_coordinator::on_raycast_response>(*this);

    create_worker();
}

island_coordinator::~island_coordinator() {
    m_worker_ctx->terminate();
}

void island_coordinator::on_construct_graph_node(entt::registry &registry, entt::entity entity) {
    if (!m_importing) {
        m_new_graph_nodes.push_back(entity);
    }
}

void island_coordinator::on_construct_graph_edge(entt::registry &registry, entt::entity entity) {
    if (!m_importing) {
        m_new_graph_edges.push_back(entity);
    }
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

    if (m_importing) return;

    // When importing delta, the entity is removed from the entity map as part
    // of the import process. Otherwise, the removal has to be done here.
    if (m_worker_ctx->m_entity_map.contains_local(entity)) {
        m_worker_ctx->m_entity_map.erase_local(entity);
    }

    // Notify the worker of the destruction which happened in the main registry
    // first.
    m_worker_ctx->m_op_builder->destroy(entity);
}

void island_coordinator::on_destroy_graph_edge(entt::registry &registry, entt::entity entity) {
    auto &edge = registry.get<graph_edge>(entity);
    auto &graph = registry.ctx().at<entity_graph>();
    graph.remove_edge(edge.edge_index);

    if (m_importing) return;

    // When importing delta, the entity is removed from the entity map as part
    // of the import process. Otherwise, the removal has to be done here.
    if (m_worker_ctx->m_entity_map.contains_local(entity)) {
        m_worker_ctx->m_entity_map.erase_local(entity);
    }

    // Notify the worker of the destruction which happened in the main registry
    // first.
    m_worker_ctx->m_op_builder->destroy(entity);
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

void island_coordinator::init_new_nodes_and_edges() {
    // Entities that were created and destroyed before a call to `edyn::update`
    // are still in these collections, thus remove invalid entities first.
    entity_vector_erase_invalid(m_new_graph_nodes, *m_registry);
    entity_vector_erase_invalid(m_new_graph_edges, *m_registry);

    if (!m_new_graph_nodes.empty()) {
        m_worker_ctx->m_op_builder->create(m_new_graph_nodes.begin(), m_new_graph_nodes.end());
        m_worker_ctx->m_op_builder->emplace_all(m_new_graph_nodes);
        m_new_graph_nodes.clear();
    }

    if (!m_new_graph_edges.empty()) {
        m_worker_ctx->m_op_builder->create(m_new_graph_edges.begin(), m_new_graph_edges.end());
        m_worker_ctx->m_op_builder->emplace_all(m_new_graph_edges);
        m_new_graph_edges.clear();
    }
}

void island_coordinator::create_worker() {
    // The `simulationworker` is dynamically allocated and kept alive while
    // the simulation runs asynchronously. The job that's created for it calls its
    // `update` function which reschedules itself to be run over and over again.
    // After the `finish` function is called on it it will be deallocated on the
    // next run.
    auto &settings = m_registry->ctx().at<edyn::settings>();
    auto &material_table = m_registry->ctx().at<edyn::material_mix_table>();
    auto *worker = new simulation_worker(settings, material_table);

    m_worker_ctx = std::make_unique<simulation_worker_context>(worker, (*settings.make_reg_op_builder)(*m_registry));
    m_worker_ctx->m_timestamp = performance_time();
}

double island_coordinator::get_simulation_timestamp() const {
    return m_worker_ctx->m_timestamp;
}

void island_coordinator::refresh_dirty_entities() {
    auto dirty_view = m_registry->view<dirty>();
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

    auto refresh = [this](entt::entity entity, dirty &dirty) {
        auto &builder = m_worker_ctx->m_op_builder;

        if (dirty.is_new_entity) {
            builder->create(entity);
        }

        builder->emplace_type_ids(entity, dirty.created_ids.begin(), dirty.created_ids.end());
        builder->replace_type_ids(entity, dirty.updated_ids.begin(), dirty.updated_ids.end());
        builder->remove_type_ids(entity, dirty.destroyed_ids.begin(), dirty.destroyed_ids.end());
    };

    dirty_view.each([&](entt::entity entity, dirty &dirty) {
        remove_unshared(dirty.created_ids);
        remove_unshared(dirty.updated_ids);
        remove_unshared(dirty.destroyed_ids);
        refresh(entity, dirty);
    });

    m_registry->clear<dirty>();
}

void island_coordinator::on_step_update(const message<msg::step_update> &msg) {
    m_importing = true;
    auto &registry = *m_registry;

    auto &ops = msg.content.ops;

    ops.execute(registry, m_worker_ctx->m_entity_map);

    // Insert entity mappings for new entities into the current op.
    ops.create_for_each([&](entt::entity remote_entity) {
        if (m_worker_ctx->m_entity_map.contains(remote_entity)) {
            auto local_entity = m_worker_ctx->m_entity_map.at(remote_entity);
            m_worker_ctx->m_op_builder->add_entity_mapping(local_entity, remote_entity);
        }
    });

    m_worker_ctx->m_timestamp = msg.content.timestamp;

    auto node_view = registry.view<graph_node>();

    // Insert nodes in the graph for each new rigid body.
    auto &graph = registry.ctx().at<entity_graph>();
    auto insert_node = [&](entt::entity remote_entity) {
        auto local_entity = m_worker_ctx->m_entity_map.at(remote_entity);
        auto non_connecting = !registry.any_of<procedural_tag>(local_entity);
        auto node_index = graph.insert_node(local_entity, non_connecting);
        registry.emplace<graph_node>(local_entity, node_index);
    };

    ops.emplace_for_each<rigidbody_tag, external_tag>(insert_node);

    // Insert edges in the graph for constraints.
    ops.emplace_for_each(constraints_tuple, [&](entt::entity remote_entity, const auto &con) {
        auto local_entity = m_worker_ctx->m_entity_map.at(remote_entity);

        if (registry.any_of<graph_edge>(local_entity)) return;

        auto [node0] = node_view.get(m_worker_ctx->m_entity_map.at(con.body[0]));
        auto [node1] = node_view.get(m_worker_ctx->m_entity_map.at(con.body[1]));
        auto edge_index = graph.insert_edge(local_entity, node0.node_index, node1.node_index);
        registry.emplace<graph_edge>(local_entity, edge_index);
    });

    m_importing = false;

    // Generate contact events.
    ops.replace_for_each<contact_manifold_events>([&](entt::entity remote_entity,
                                                      const contact_manifold_events &events) {
        if (!m_worker_ctx->m_entity_map.contains(remote_entity)) {
            return;
        }

        auto manifold_entity = m_worker_ctx->m_entity_map.at(remote_entity);

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

    (*g_mark_replaced_network_dirty)(registry, ops, m_worker_ctx->m_entity_map, m_timestamp);
}

void island_coordinator::on_raycast_response(const message<msg::raycast_response> &msg) {
    auto &res = msg.content;
    auto &ctx = m_raycast_ctx.at(res.id);

    if (res.result.fraction < ctx.result.fraction) {
        ctx.result = res.result;
        ctx.result.entity = m_worker_ctx->m_entity_map.at(ctx.result.entity);
    }

    ctx.delegate(res.id, ctx.result, ctx.p0, ctx.p1);
    m_raycast_ctx.erase(res.id);
}

void island_coordinator::sync() {
    if (!m_worker_ctx->reg_ops_empty()) {
        m_worker_ctx->send_reg_ops(m_message_queue_handle.identifier);
    }

    m_worker_ctx->flush();
}

void island_coordinator::update() {
    m_timestamp = performance_time();
    init_new_nodes_and_edges();

    // Insert dirty components into the registry ops before importing messages,
    // which could override the state of these components that have been modified
    // by the user.

    m_message_queue_handle.update();

    refresh_dirty_entities();
    init_new_nodes_and_edges();
    sync();
}

void island_coordinator::set_paused(bool paused) {
    m_worker_ctx->send<msg::set_paused>(m_message_queue_handle.identifier, paused);
}

void island_coordinator::step_simulation() {
    m_worker_ctx->send<msg::step_simulation>(m_message_queue_handle.identifier);
}

void island_coordinator::settings_changed() {
    auto &settings = m_registry->ctx().at<edyn::settings>();
    m_worker_ctx->send<msg::set_settings>(m_message_queue_handle.identifier, settings);
}

void island_coordinator::material_table_changed() {
    auto &material_table = m_registry->ctx().at<material_mix_table>();
    m_worker_ctx->send<msg::set_material_table>(m_message_queue_handle.identifier, material_table);
}

void island_coordinator::set_center_of_mass(entt::entity entity, const vector3 &com) {
    m_worker_ctx->send<msg::set_com>(m_message_queue_handle.identifier, entity, com);
}

}
