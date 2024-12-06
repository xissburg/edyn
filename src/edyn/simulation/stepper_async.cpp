#include "edyn/simulation/stepper_async.hpp"
#include "edyn/collision/contact_event_emitter.hpp"
#include "edyn/collision/contact_manifold_events.hpp"
#include "edyn/collision/query_aabb.hpp"
#include "edyn/comp/child_list.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/constraints/null_constraint.hpp"
#include "edyn/constraints/constraint.hpp"
#include "edyn/context/registry_operation_context.hpp"
#include "edyn/math/math.hpp"
#include "edyn/parallel/message.hpp"
#include "edyn/sys/update_presentation.hpp"
#include "edyn/core/entity_graph.hpp"
#include "edyn/comp/graph_node.hpp"
#include "edyn/comp/graph_edge.hpp"
#include "edyn/replication/registry_operation.hpp"
#include "edyn/context/settings.hpp"
#include "edyn/dynamics/material_mixing.hpp"
#include "edyn/util/constraint_util.hpp"
#include <entt/entity/registry.hpp>
#include <numeric>

namespace edyn {

stepper_async::stepper_async(entt::registry &registry, double time)
    : m_registry(&registry)
    , m_message_queue_handle(
        message_dispatcher::global().make_queue<
            msg::step_update,
            msg::raycast_response,
            msg::query_aabb_response
        >("main"))
    , m_worker(registry.ctx().get<settings>(),
               registry.ctx().get<registry_operation_context>(),
               registry.ctx().get<material_mix_table>())
    , m_last_time(time)
    , m_sim_time(time)
{
    m_connections.push_back(registry.on_construct<graph_node>().connect<&stepper_async::on_construct_shared>(*this));
    m_connections.push_back(registry.on_destroy<graph_node>().connect<&stepper_async::on_destroy_graph_node>(*this));
    m_connections.push_back(registry.on_construct<graph_edge>().connect<&stepper_async::on_construct_shared>(*this));
    m_connections.push_back(registry.on_destroy<graph_edge>().connect<&stepper_async::on_destroy_graph_edge>(*this));
    m_connections.push_back(registry.on_construct<child_list>().connect<&stepper_async::on_construct_shared>(*this));

    m_message_queue_handle.sink<msg::step_update>().connect<&stepper_async::on_step_update>(*this);
    m_message_queue_handle.sink<msg::raycast_response>().connect<&stepper_async::on_raycast_response>(*this);
    m_message_queue_handle.sink<msg::query_aabb_response>().connect<&stepper_async::on_query_aabb_response>(*this);

    auto &reg_op_ctx = m_registry->ctx().get<registry_operation_context>();
    m_op_builder = (*reg_op_ctx.make_reg_op_builder)(*m_registry);
    m_op_observer = (*reg_op_ctx.make_reg_op_observer)(*m_op_builder);

    m_worker.start();
}

void stepper_async::on_construct_shared(entt::registry &registry, entt::entity entity) {
    m_op_observer->observe(entity);
}

void stepper_async::on_destroy_graph_node(entt::registry &registry, entt::entity entity) {
    auto &node = registry.get<graph_node>(entity);
    auto &graph = registry.ctx().get<entity_graph>();

    // Prevent edges from being removed in `on_destroy_graph_edge`. The more
    // direct `entity_graph::remove_all_edges` will be used instead.
    registry.on_destroy<graph_edge>().disconnect<&stepper_async::on_destroy_graph_edge>(*this);

    graph.visit_edges(node.node_index, [&](auto edge_index) {
        auto edge_entity = graph.edge_entity(edge_index);
        registry.destroy(edge_entity);
        m_op_observer->unobserve(edge_entity);

        if (!m_importing) {
            if (m_entity_map.contains_local(edge_entity)) {
                m_entity_map.erase_local(edge_entity);
            }
        }
    });

    registry.on_destroy<graph_edge>().connect<&stepper_async::on_destroy_graph_edge>(*this);

    graph.remove_all_edges(node.node_index);
    graph.remove_node(node.node_index);
    m_op_observer->unobserve(entity);

    if (!m_importing) {
        // When importing delta, the entity is removed from the entity map as part
        // of the import process. Otherwise, the removal has to be done here.
        if (m_entity_map.contains_local(entity)) {
            m_entity_map.erase_local(entity);
        }
    }
}

void stepper_async::on_destroy_graph_edge(entt::registry &registry, entt::entity entity) {
    auto &edge = registry.get<graph_edge>(entity);
    auto &graph = registry.ctx().get<entity_graph>();
    graph.remove_edge(edge.edge_index);
    m_op_observer->unobserve(entity);

    if (!m_importing) {
        if (m_entity_map.contains_local(entity)) {
            m_entity_map.erase_local(entity);
        }
    }
}

void stepper_async::on_step_update(message<msg::step_update> &msg) {
    auto &registry = *m_registry;
    auto &graph = registry.ctx().get<entity_graph>();

    m_importing = true;
    m_op_observer->set_active(false);

    m_sim_time = msg.content.timestamp;
    // Only calculate delay if the sim time was set.
    m_should_calculate_presentation_delay = true;

    auto &ops = msg.content.ops;
    ops.execute(registry, m_entity_map, [&](operation_base *op) {
        auto op_type = op->operation_type();
        auto remote_entity = op->entity;

        // Insert entity mappings for new entities into the current op.
        if (op_type == registry_operation_type::create) {
            auto local_entity = m_entity_map.at(remote_entity);
            m_op_builder->add_entity_mapping(local_entity, remote_entity);
        }

        // Insert nodes in the graph for each new rigid body.
        if (op_type == registry_operation_type::emplace && op->payload_type_any_of<rigidbody_tag, external_tag>()) {
            auto local_entity = m_entity_map.at(remote_entity);
            auto non_connecting = !registry.any_of<procedural_tag>(local_entity);
            auto node_index = graph.insert_node(local_entity, non_connecting);
            registry.emplace<graph_node>(local_entity, node_index);

            if (non_connecting) {
                // `multi_island_resident` is not a shared component thus add it
                // manually here.
                registry.emplace<multi_island_resident>(local_entity);
            }
        }

        // Insert edges in the graph for constraints.
        if (op_type == registry_operation_type::emplace &&
           (op->payload_type_any_of(constraints_tuple) || op->payload_type_any_of<null_constraint>()))
        {
            auto local_entity = m_entity_map.at(remote_entity);

            // There could be multiple constraints (of different types) assigned to
            // the same entity, which means it could already have an edge.
            if (!registry.any_of<graph_edge>(local_entity)) {
                create_graph_edge_for_constraints(registry, local_entity, graph, constraints_tuple);
                create_graph_edge_for_constraint<null_constraint>(registry, local_entity, graph);
            }
        }
    });

    m_importing = false;
    m_op_observer->set_active(true);

    // Must consume events after each snapshot to avoid losing any event that
    // could be overriden in the next snapshot.
    auto &emitter = registry.ctx().get<contact_event_emitter>();
    emitter.consume_events();
}

void stepper_async::on_raycast_response(message<msg::raycast_response> &msg) {
    auto &response = msg.content;
    auto result = response.result;

    if (result.entity != entt::null) {
        if (m_entity_map.contains(result.entity)) {
            result.entity = m_entity_map.at(result.entity);
        } else {
            result.entity = entt::null;
        }
    }

    auto &ctx = m_raycast_ctx.at(response.id);
    ctx.delegate(response.id, result, ctx.p0, ctx.p1);
    m_raycast_ctx.erase(response.id);
}

void stepper_async::on_query_aabb_response(message<msg::query_aabb_response> &msg) {
    auto &response = msg.content;
    auto result = query_aabb_result{};
    auto &emap = m_entity_map;

    for (auto entity : response.island_entities) {
        if (emap.contains(entity)) {
            result.island_entities.push_back(emap.at(entity));
        }
    }

    for (auto entity : response.procedural_entities) {
        if (emap.contains(entity)) {
            result.procedural_entities.push_back(emap.at(entity));
        }
    }

    for (auto entity : response.non_procedural_entities) {
        if (emap.contains(entity)) {
            result.non_procedural_entities.push_back(emap.at(entity));
        }
    }

    auto &ctx = m_query_aabb_ctx.at(response.id);
    ctx.delegate(response.id, std::move(result));
    m_query_aabb_ctx.erase(response.id);
}

void stepper_async::sync() {
    if (!m_op_builder->empty()) {
        send_message_to_worker<msg::update_entities>(m_op_builder->finish());
    }
}

void stepper_async::calculate_presentation_delay(double current_time, double elapsed, scalar fixed_dt) {
    // Keep a history of differences between current time and simulation time.
    // Adjust presentation delay to keep it close to the highest time difference,
    // with the goal of having the presentation interpolation happen backwards,
    // i.e. avoid extrapolation from happening instead, which causes jitter.
    std::rotate(m_time_diff_samples.begin(), m_time_diff_samples.begin() + 1, m_time_diff_samples.end());
    auto time_diff = std::min(current_time - m_sim_time, 1.0);
    m_time_diff_samples.back() = time_diff;

    auto time_diff_avg = std::accumulate(m_time_diff_samples.begin(), m_time_diff_samples.end(), 0.0) / m_time_diff_samples.size();
    auto time_diff_dev = m_time_diff_samples; // Calculate deviation from average.

    for (auto &val : time_diff_dev) {
        val = std::abs(val - time_diff_avg);
    }

    // Average absolute deviation of time differences.
    auto time_diff_dev_avg = std::accumulate(time_diff_dev.begin(), time_diff_dev.end(), 0.0) / time_diff_dev.size();
    // Keep presentation delay in a fixed_dt boundary.
    auto target_presentation_delay = std::ceil((time_diff_avg + time_diff_dev_avg) / fixed_dt) * fixed_dt;
    auto presentation_error = target_presentation_delay - m_presentation_delay;

    if (!m_adjusting_presentation_delay) {
        // Only start adjusting if the target moves away significantly.
        m_adjusting_presentation_delay = std::abs(presentation_error) > fixed_dt;
    }

    if (m_adjusting_presentation_delay) {
        // Correct faster if the error grows, which would lead to extrapolation.
        auto rate = presentation_error > 0 ? 5 : 2;
        m_presentation_delay += presentation_error * std::min(rate * elapsed, 1.0);

        if (std::abs(target_presentation_delay - m_presentation_delay) < 0.00001) {
            m_presentation_delay = target_presentation_delay;
            m_adjusting_presentation_delay = false;
        }
    }
}

void stepper_async::update(double current_time) {
    m_message_queue_handle.update();
    sync();

    auto &settings = m_registry->ctx().get<edyn::settings>();
    if (settings.clear_actions_func) {
        (*settings.clear_actions_func)(*m_registry);
    }

    if (m_paused) {
        snap_presentation(*m_registry);
    } else {
        const auto elapsed = std::min(current_time - m_last_time, 1.0);

        if (m_should_calculate_presentation_delay) {
            calculate_presentation_delay(current_time, elapsed, settings.fixed_dt);
        }

        update_presentation(*m_registry, m_sim_time, current_time, elapsed, m_presentation_delay);
    }

    m_should_calculate_presentation_delay = false;
    m_last_time = current_time;
}

void stepper_async::set_paused(bool paused) {
    m_paused = paused;
    m_presentation_delay = 0;
    m_time_diff_samples = {};
    m_adjusting_presentation_delay = true;
    send_message_to_worker<msg::set_paused>(paused);
}

void stepper_async::step_simulation() {
    send_message_to_worker<msg::step_simulation>();
}

void stepper_async::settings_changed() {
    auto &settings = m_registry->ctx().get<edyn::settings>();
    send_message_to_worker<msg::set_settings>(settings);
}

void stepper_async::reg_op_ctx_changed() {
    auto &reg_op_ctx = m_registry->ctx().get<registry_operation_context>();
    m_op_builder = (*reg_op_ctx.make_reg_op_builder)(*m_registry);
    m_op_observer = (*reg_op_ctx.make_reg_op_observer)(*m_op_builder);
    send_message_to_worker<msg::set_registry_operation_context>(reg_op_ctx);
}

void stepper_async::material_table_changed() {
    auto &material_table = m_registry->ctx().get<material_mix_table>();
    send_message_to_worker<msg::set_material_table>(material_table);
}

void stepper_async::set_center_of_mass(entt::entity entity, const vector3 &com) {
    send_message_to_worker<msg::set_com>(entity, com);
}

void stepper_async::wake_up_entity(entt::entity entity) {
    auto msg = std::vector<entt::entity>{};
    msg.push_back(entity);
    send_message_to_worker<msg::wake_up_residents>(std::move(msg));
}

void stepper_async::set_rigidbody_kind(entt::entity entity, rigidbody_kind kind) {
    // Must sync existing component changes to ensure the simulation registry
    // contains the latest changes before processing the `change_rigidbody_kind` message.
    sync();
    auto msg = msg::change_rigidbody_kind{};
    msg.changes.emplace_back(entity, kind);
    send_message_to_worker<msg::change_rigidbody_kind>(std::move(msg));
}

raycast_id_type stepper_async::raycast(vector3 p0, vector3 p1,
                                       const raycast_delegate_type &delegate,
                                       std::vector<entt::entity> ignore_entities) {
    auto id = m_next_raycast_id++;
    auto &ctx = m_raycast_ctx[id];
    ctx.delegate = delegate;
    ctx.p0 = p0;
    ctx.p1 = p1;
    send_message_to_worker<msg::raycast_request>(id, p0, p1, ignore_entities);

    return id;
}

query_aabb_id_type stepper_async::query_aabb(const AABB &aabb,
                                             const query_aabb_delegate_type &delegate,
                                             bool query_procedural,
                                             bool query_non_procedural,
                                             bool query_islands) {
    auto id = m_next_query_aabb_id++;
    auto &ctx = m_query_aabb_ctx[id];
    ctx.delegate = delegate;
    ctx.aabb = aabb;
    send_message_to_worker<msg::query_aabb_request>(id, aabb, query_procedural, query_non_procedural, query_islands);

    return id;
}

query_aabb_id_type stepper_async::query_aabb_of_interest(const AABB &aabb,
                                                         const query_aabb_delegate_type &delegate) {
    auto id = m_next_query_aabb_id++;
    auto &ctx = m_query_aabb_ctx[id];
    ctx.delegate = delegate;
    ctx.aabb = aabb;
    send_message_to_worker<msg::query_aabb_of_interest_request>(id, aabb);

    return id;
}

}
