#include "edyn/simulation/stepper_async.hpp"
#include "edyn/collision/contact_event_emitter.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/collision/contact_manifold_events.hpp"
#include "edyn/collision/contact_point.hpp"
#include "edyn/collision/dynamic_tree.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/comp/shape_index.hpp"
#include "edyn/constraints/null_constraint.hpp"
#include "edyn/context/registry_operation_context.hpp"
#include "edyn/networking/networking_external.hpp"
#include "edyn/parallel/message.hpp"
#include "edyn/replication/component_index_source.hpp"
#include "edyn/shapes/shapes.hpp"
#include "edyn/config/config.h"
#include "edyn/constraints/constraint.hpp"
#include "edyn/constraints/contact_constraint.hpp"
#include "edyn/simulation/simulation_worker.hpp"
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

stepper_async::stepper_async(entt::registry &registry)
    : m_registry(&registry)
    , m_message_queue_handle(
        message_dispatcher::global().make_queue<
            msg::step_update,
            msg::raycast_response,
            msg::query_aabb_response
        >("main"))
{
    registry.on_construct<graph_node>().connect<&stepper_async::on_construct_graph_node>(*this);
    registry.on_destroy<graph_node>().connect<&stepper_async::on_destroy_graph_node>(*this);
    registry.on_construct<graph_edge>().connect<&stepper_async::on_construct_graph_edge>(*this);
    registry.on_destroy<graph_edge>().connect<&stepper_async::on_destroy_graph_edge>(*this);

    m_message_queue_handle.sink<msg::step_update>().connect<&stepper_async::on_step_update>(*this);
    m_message_queue_handle.sink<msg::raycast_response>().connect<&stepper_async::on_raycast_response>(*this);
    m_message_queue_handle.sink<msg::query_aabb_response>().connect<&stepper_async::on_query_aabb_response>(*this);

    auto &reg_op_ctx = m_registry->ctx().at<registry_operation_context>();
    m_op_builder = (*reg_op_ctx.make_reg_op_builder)(*m_registry);
    m_op_observer = (*reg_op_ctx.make_reg_op_observer)(*m_op_builder);

    create_worker();
}

stepper_async::~stepper_async() {
    m_registry->on_construct<graph_node>().disconnect<&stepper_async::on_construct_graph_node>(*this);
    m_registry->on_destroy<graph_node>().disconnect<&stepper_async::on_destroy_graph_node>(*this);
    m_registry->on_construct<graph_edge>().disconnect<&stepper_async::on_construct_graph_edge>(*this);
    m_registry->on_destroy<graph_edge>().disconnect<&stepper_async::on_destroy_graph_edge>(*this);

    m_worker_ctx->terminate();
}

void stepper_async::on_construct_graph_node(entt::registry &registry, entt::entity entity) {
    registry.emplace<shared_tag>(entity);
}

void stepper_async::on_construct_graph_edge(entt::registry &registry, entt::entity entity) {
    registry.emplace<shared_tag>(entity);
}

void stepper_async::on_destroy_graph_node(entt::registry &registry, entt::entity entity) {
    auto &node = registry.get<graph_node>(entity);
    auto &graph = registry.ctx().at<entity_graph>();

    // Prevent edges from being removed in `on_destroy_graph_edge`. The more
    // direct `entity_graph::remove_all_edges` will be used instead.
    registry.on_destroy<graph_edge>().disconnect<&stepper_async::on_destroy_graph_edge>(*this);

    graph.visit_edges(node.node_index, [&](auto edge_index) {
        auto edge_entity = graph.edge_entity(edge_index);
        registry.destroy(edge_entity);

        if (!m_importing) {
            if (m_worker_ctx->m_entity_map.contains_local(edge_entity)) {
                m_worker_ctx->m_entity_map.erase_local(edge_entity);
            }
        }
    });

    registry.on_destroy<graph_edge>().connect<&stepper_async::on_destroy_graph_edge>(*this);

    graph.remove_all_edges(node.node_index);
    graph.remove_node(node.node_index);

    if (!m_importing) {
        // When importing delta, the entity is removed from the entity map as part
        // of the import process. Otherwise, the removal has to be done here.
        if (m_worker_ctx->m_entity_map.contains_local(entity)) {
            m_worker_ctx->m_entity_map.erase_local(entity);
        }
    }
}

void stepper_async::on_destroy_graph_edge(entt::registry &registry, entt::entity entity) {
    auto &edge = registry.get<graph_edge>(entity);
    auto &graph = registry.ctx().at<entity_graph>();
    graph.remove_edge(edge.edge_index);

    if (!m_importing) {
        if (m_worker_ctx->m_entity_map.contains_local(entity)) {
            m_worker_ctx->m_entity_map.erase_local(entity);
        }
    }
}

void stepper_async::create_worker() {
    // The `simulation_worker` is dynamically allocated and kept alive while
    // the simulation runs asynchronously. The job that's created for it calls its
    // `update` function which reschedules itself to be run over and over again.
    // After the `finish` function is called on it it will be deallocated on the
    // next run.
    auto &settings = m_registry->ctx().at<edyn::settings>();
    auto &reg_op_ctx = m_registry->ctx().at<registry_operation_context>();
    auto &material_table = m_registry->ctx().at<edyn::material_mix_table>();
    auto *worker = new simulation_worker(settings, reg_op_ctx, material_table);

    m_worker_ctx = std::make_unique<simulation_worker_context>(worker);
    m_worker_ctx->m_timestamp = performance_time();
}

double stepper_async::get_simulation_timestamp() const {
    return m_worker_ctx->m_timestamp;
}

void stepper_async::on_step_update(const message<msg::step_update> &msg) {
    m_importing = true;
    m_op_observer->set_active(false);

    auto &registry = *m_registry;
    auto &ops = msg.content.ops;
    ops.execute(registry, m_worker_ctx->m_entity_map);

    // Insert entity mappings for new entities into the current op.
    ops.create_for_each([&](entt::entity remote_entity) {
        if (m_worker_ctx->m_entity_map.contains(remote_entity)) {
            auto local_entity = m_worker_ctx->m_entity_map.at(remote_entity);
            m_op_builder->add_entity_mapping(local_entity, remote_entity);
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
    auto insert_edge = [&](entt::entity remote_entity, const auto &con) {
        auto local_entity = m_worker_ctx->m_entity_map.at(remote_entity);

        // There could be multiple constraints (of different types) assigned to
        // the same entity, which means it could already have an edge.
        if (registry.any_of<graph_edge>(local_entity)) return;

        auto [node0] = node_view.get(m_worker_ctx->m_entity_map.at(con.body[0]));
        auto [node1] = node_view.get(m_worker_ctx->m_entity_map.at(con.body[1]));
        auto edge_index = graph.insert_edge(local_entity, node0.node_index, node1.node_index);
        registry.emplace<graph_edge>(local_entity, edge_index);
    };
    ops.emplace_for_each(constraints_tuple, insert_edge);
    ops.emplace_for_each<null_constraint>(insert_edge);

    m_importing = false;
    m_op_observer->set_active(true);

    // Must consume events after each snapshot to avoid losing any event that
    // could be overriden in the next snapshot.
    auto &emitter = registry.ctx().at<contact_event_emitter>();
    emitter.consume_events();

    (*g_mark_replaced_network_dirty)(registry, ops, m_worker_ctx->m_entity_map, m_worker_ctx->m_timestamp);
}

void stepper_async::on_raycast_response(const message<msg::raycast_response> &msg) {
    auto &res = msg.content;
    auto &ctx = m_raycast_ctx.at(res.id);

    if (res.result.fraction < ctx.result.fraction) {
        ctx.result = res.result;
        ctx.result.entity = m_worker_ctx->m_entity_map.at(ctx.result.entity);
    }

    ctx.delegate(res.id, ctx.result, ctx.p0, ctx.p1);
    m_raycast_ctx.erase(res.id);
}

void stepper_async::on_query_aabb_response(const message<msg::query_aabb_response> &msg) {
    auto &res = msg.content;
    auto &ctx = m_query_aabb_ctx.at(res.id);
    ctx.delegate(res.id, ctx.result);
    m_query_aabb_ctx.erase(res.id);
}

void stepper_async::sync() {
    if (!m_op_builder->empty()) {
        m_worker_ctx->send<msg::update_entities>(m_message_queue_handle.identifier, m_op_builder->finish());
    }

    m_worker_ctx->flush();
}

void stepper_async::update() {
    m_message_queue_handle.update();
    sync();
}

void stepper_async::set_paused(bool paused) {
    m_worker_ctx->send<msg::set_paused>(m_message_queue_handle.identifier, paused);
}

void stepper_async::step_simulation() {
    m_worker_ctx->send<msg::step_simulation>(m_message_queue_handle.identifier);
}

void stepper_async::settings_changed() {
    auto &settings = m_registry->ctx().at<edyn::settings>();
    m_worker_ctx->send<msg::set_settings>(m_message_queue_handle.identifier, settings);
}

void stepper_async::reg_op_ctx_changed() {
    auto &reg_op_ctx = m_registry->ctx().at<registry_operation_context>();
    m_op_builder = (*reg_op_ctx.make_reg_op_builder)(*m_registry);
    m_op_observer = (*reg_op_ctx.make_reg_op_observer)(*m_op_builder);
    m_worker_ctx->send<msg::set_registry_operation_context>(m_message_queue_handle.identifier, reg_op_ctx);
}

void stepper_async::material_table_changed() {
    auto &material_table = m_registry->ctx().at<material_mix_table>();
    m_worker_ctx->send<msg::set_material_table>(m_message_queue_handle.identifier, material_table);
}

void stepper_async::set_center_of_mass(entt::entity entity, const vector3 &com) {
    m_worker_ctx->send<msg::set_com>(m_message_queue_handle.identifier, entity, com);
}

raycast_id_type stepper_async::raycast(vector3 p0, vector3 p1,
                                       const raycast_delegate_type &delegate,
                                       std::vector<entt::entity> ignore_entities) {
    auto id = m_next_raycast_id++;
    auto &ctx = m_raycast_ctx[id];
    ctx.delegate = delegate;
    ctx.p0 = p0;
    ctx.p1 = p1;
    m_worker_ctx->send<msg::raycast_request>(m_message_queue_handle.identifier, id, p0, p1, ignore_entities);

    return id;
}

query_aabb_id_type stepper_async::query_island_aabb(const AABB &aabb,
                                                    const query_aabb_delegate_type &delegate) {
    return query_aabb(aabb, delegate, true);
}

query_aabb_id_type stepper_async::query_aabb(const AABB &aabb,
                                             const query_aabb_delegate_type &delegate,
                                             bool islands_only) {
    auto id = m_next_query_aabb_id++;
    auto &ctx = m_query_aabb_ctx[id];
    ctx.delegate = delegate;
    ctx.aabb = aabb;
    m_worker_ctx->send<msg::query_aabb_request>(m_message_queue_handle.identifier, id, aabb, islands_only);

    return id;
}

}
