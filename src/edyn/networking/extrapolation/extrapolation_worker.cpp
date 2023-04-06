#include "edyn/networking/extrapolation/extrapolation_worker.hpp"
#include "edyn/collision/broadphase.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/collision/narrowphase.hpp"
#include "edyn/collision/contact_manifold_map.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/rotated_mesh_list.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/config/config.h"
#include "edyn/constraints/contact_constraint.hpp"
#include "edyn/context/registry_operation_context.hpp"
#include "edyn/context/settings.hpp"
#include "edyn/dynamics/material_mixing.hpp"
#include "edyn/networking/context/client_network_context.hpp"
#include "edyn/networking/extrapolation/extrapolation_request.hpp"
#include "edyn/networking/util/input_state_history.hpp"
#include "edyn/parallel/message.hpp"
#include "edyn/replication/registry_operation.hpp"
#include "edyn/replication/registry_operation_observer.hpp"
#include "edyn/serialization/memory_archive.hpp"
#include "edyn/core/entity_graph.hpp"
#include "edyn/replication/registry_operation_builder.hpp"
#include "edyn/comp/graph_node.hpp"
#include "edyn/comp/graph_edge.hpp"
#include "edyn/sys/update_aabbs.hpp"
#include "edyn/sys/update_inertias.hpp"
#include "edyn/sys/update_origins.hpp"
#include "edyn/sys/update_rotated_meshes.hpp"
#include "edyn/parallel/job_dispatcher.hpp"
#include "edyn/math/transform.hpp"
#include "edyn/time/time.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

extrapolation_worker::extrapolation_worker(const settings &settings,
                                           const registry_operation_context &reg_op_ctx,
                                           const material_mix_table &material_table,
                                           std::shared_ptr<input_state_history> input_history,
                                           make_extrapolation_modified_comp_func_t *make_extrapolation_modified_comp)
    : m_solver(m_registry)
    , m_input_history(input_history)
    , m_poly_initializer(m_registry)
    , m_island_manager(m_registry)
    , m_message_queue(message_dispatcher::global().make_queue<
        extrapolation_request,
        registry_operation,
        msg::set_settings,
        msg::set_registry_operation_context,
        msg::set_material_table,
        msg::set_extrapolator_context_settings>("extrapolation_worker"))
{
    m_registry.ctx().emplace<contact_manifold_map>(m_registry);
    m_registry.ctx().emplace<broadphase>(m_registry);
    m_registry.ctx().emplace<narrowphase>(m_registry);
    m_registry.ctx().emplace<entity_graph>();
    m_registry.ctx().emplace<edyn::settings>(settings);
    m_registry.ctx().emplace<registry_operation_context>(reg_op_ctx);
    m_registry.ctx().emplace<material_mix_table>(material_table);

    m_message_queue.sink<extrapolation_request>().connect<&extrapolation_worker::on_extrapolation_request>(*this);
    m_message_queue.sink<registry_operation>().connect<&extrapolation_worker::on_registry_operation>(*this);
    m_message_queue.sink<msg::set_settings>().connect<&extrapolation_worker::on_set_settings>(*this);
    m_message_queue.sink<msg::set_registry_operation_context>().connect<&extrapolation_worker::on_set_reg_op_ctx>(*this);
    m_message_queue.sink<msg::set_material_table>().connect<&extrapolation_worker::on_set_material_table>(*this);
    m_message_queue.sink<msg::set_extrapolator_context_settings>().connect<&extrapolation_worker::on_set_extrapolator_context_settings>(*this);
    m_message_queue.push_sink().connect<&extrapolation_worker::on_push_message>(*this);

    m_modified_comp = (*make_extrapolation_modified_comp)(m_registry);
}

extrapolation_worker::~extrapolation_worker() {
    stop();
}

void extrapolation_worker::init() {
    auto &settings = m_registry.ctx().at<edyn::settings>();
    if (settings.init_callback) {
        (*settings.init_callback)(m_registry);
    }
}

void extrapolation_worker::deinit() {
    auto &settings = m_registry.ctx().at<edyn::settings>();
    if (settings.deinit_callback) {
        (*settings.deinit_callback)(m_registry);
    }
}

void extrapolation_worker::start() {
    EDYN_ASSERT(!m_thread);
    m_running.store(true, std::memory_order_release);
    m_thread = std::make_unique<std::thread>(&extrapolation_worker::run, this);
}

void extrapolation_worker::stop() {
    EDYN_ASSERT(m_thread);
    m_running.store(false, std::memory_order_release);
    m_cv.notify_one();
    m_thread->join();
    m_thread.reset();
}

void extrapolation_worker::set_settings(const edyn::settings &settings) {
    auto &dispatcher = message_dispatcher::global();
    dispatcher.send<msg::set_settings>(m_message_queue.identifier, {"unknown"}, settings);
}

void extrapolation_worker::set_material_table(const material_mix_table &material_table) {
    auto &dispatcher = message_dispatcher::global();
    dispatcher.send<msg::set_material_table>(m_message_queue.identifier, {"unknown"}, material_table);
}

void extrapolation_worker::set_registry_operation_context(const registry_operation_context &reg_op_ctx) {
    auto &dispatcher = message_dispatcher::global();
    dispatcher.send<msg::set_registry_operation_context>(m_message_queue.identifier, {"unknown"}, reg_op_ctx);
}

void extrapolation_worker::set_context_settings(std::shared_ptr<input_state_history> input_history,
                                                make_extrapolation_modified_comp_func_t *make_extrapolation_modified_comp) {
    EDYN_ASSERT(make_extrapolation_modified_comp != nullptr);
    auto &dispatcher = message_dispatcher::global();
    dispatcher.send<msg::set_extrapolator_context_settings>(m_message_queue.identifier, {"unknown"},
                                                            input_history, make_extrapolation_modified_comp);
}

void extrapolation_worker::on_extrapolation_request(message<extrapolation_request> &msg) {
    m_request = std::move(msg.content);
    m_has_work = true;
}

void extrapolation_worker::on_registry_operation(message<registry_operation> &msg) {
    auto &ops = msg.content;
    auto &emap = m_entity_map;

    // Process destroyed entities before executing because entity mappings
    // will be removed.
    for (auto remote_entity : ops.destroy_entities) {
        auto local_entity = emap.at(remote_entity);
        m_modified_comp->remove_entity(local_entity);
    }

    ops.execute(m_registry, m_entity_map);

    auto &graph = m_registry.ctx().at<entity_graph>();
    auto node_view = m_registry.view<graph_node>();
    auto procedural_view = m_registry.view<procedural_tag>();

    // Insert nodes in the graph for rigid bodies and external entities, and
    // edges for constraints, because `graph_node` and `graph_edge` are not
    // shared components.
    ops.emplace_for_each<rigidbody_tag, external_tag>([&](entt::entity remote_entity) {
        auto local_entity = emap.at(remote_entity);
        auto procedural = procedural_view.contains(local_entity);
        auto node_index = graph.insert_node(local_entity, !procedural);
        m_registry.emplace<graph_node>(local_entity, node_index);

        if (!procedural) {
            // `multi_island_resident` is not a shared component thus add it
            // manually here.
            m_registry.emplace<multi_island_resident>(local_entity);
        }
    });

    auto insert_edge = [&](entt::entity remote_entity, const auto &con) {
        auto local_entity = emap.at(remote_entity);

        // There could be multiple constraints (of different types) assigned to
        // the same entity, which means it could already have an edge.
        if (m_registry.any_of<graph_edge>(local_entity)) return;

        auto &node0 = node_view.get<graph_node>(emap.at(con.body[0]));
        auto &node1 = node_view.get<graph_node>(emap.at(con.body[1]));
        auto edge_index = graph.insert_edge(local_entity, node0.node_index, node1.node_index);
        m_registry.emplace<graph_edge>(local_entity, edge_index);
    };
    ops.emplace_for_each(constraints_tuple, insert_edge);
    ops.emplace_for_each<null_constraint>(insert_edge);

    // Initialize shapes for new entities.
    m_poly_initializer.init_new_shapes();

    // All new entities must be disabled when created.
    for (auto remote_entity : ops.create_entities) {
        auto local_entity = emap.at(remote_entity);
        m_registry.emplace<disabled_tag>(local_entity);
        m_modified_comp->add_entity(local_entity);
    }

    // Store copy of imported state into "server" state storage.
    if (!ops.create_entities.empty()) {
        entt::sparse_set entities;

        for (auto remote_entity : ops.create_entities) {
            auto local_entity = emap.at(remote_entity);
            entities.emplace(local_entity);
        }

        m_modified_comp->export_remote_state(entities);
    }
}

void extrapolation_worker::on_set_settings(message<msg::set_settings> &msg) {
    m_registry.ctx().at<settings>() = msg.content.settings;
}

void extrapolation_worker::on_set_reg_op_ctx(message<msg::set_registry_operation_context> &msg) {
    m_registry.ctx().at<registry_operation_context>() = msg.content.ctx;
}

void extrapolation_worker::on_set_material_table(message<msg::set_material_table> &msg) {
    m_registry.ctx().at<material_mix_table>() = msg.content.table;
}

void extrapolation_worker::on_set_extrapolator_context_settings(message<msg::set_extrapolator_context_settings> &msg) {
    m_input_history = msg.content.input_history;
    m_modified_comp = (*msg.content.make_extrapolation_modified_comp)(m_registry);
}

void extrapolation_worker::on_push_message() {
    m_has_messages.store(true, std::memory_order_release);
    m_cv.notify_one();
}

void extrapolation_worker::apply_history() {
    auto &settings = m_registry.ctx().at<edyn::settings>();
    auto since_time = m_current_time - settings.fixed_dt;
    m_input_history->import_each(since_time, settings.fixed_dt, m_registry, m_entity_map);
}

void extrapolation_worker::init_extrapolation() {
    m_init_time = performance_time();
    m_current_time = m_request.start_time;
    m_step_count = 0;
    m_island_manager.set_last_time(m_current_time);
    m_terminated_early = false;

    // Initialize new nodes and edges and create islands.
    m_island_manager.update(m_current_time);

    auto &graph = m_registry.ctx().at<entity_graph>();
    std::set<entity_graph::index_type> node_indices;
    auto node_view = m_registry.view<graph_node>();
    auto snapshot_entities = entt::sparse_set{};

    // The snapshot entities only include those that have a component that
    // changed recently. Though the extrapolation must include all entities
    // that belong in the same island because entities cannot be simulated in
    // isolation from their island. An island is always simulated as one unit.
    for (auto remote_entity : m_request.snapshot.entities) {
        auto local_entity = m_entity_map.at(remote_entity);
        snapshot_entities.emplace(local_entity);

        if (node_view.contains(local_entity)) {
            auto node_index = node_view.get<graph_node>(local_entity).node_index;

            if (graph.is_connecting_node(node_index)) {
                node_indices.insert(node_index);
            }
        }
    }

    // Collection of entities in all involved islands.
    auto entities = entt::sparse_set{};

    graph.reach(
        node_indices.begin(), node_indices.end(),
        [&](entt::entity entity) {
            if (!entities.contains(entity)) {
                entities.emplace(entity);
            }
        }, [&](entt::entity entity) {
            if (!entities.contains(entity)) {
                entities.emplace(entity);
            }
        }, [](auto) { return true; }, []() {});

    // Enable simulation for all involved entities.
    for (auto entity : entities) {
        m_registry.erase<disabled_tag>(entity);
    }

    // Apply last known remote state as the initial state for extrapolation.
    m_modified_comp->import_remote_state(entities);

    // Start observing changes before replacing snapshot contents into registry
    // to ensure these changes will be included in the result.
    m_modified_comp->set_observe_changes(true);

    // Replace client component state by latest server state. The snapshot
    // only contains components which have changed since the last update.
    for (auto &pool : m_request.snapshot.pools) {
        pool.ptr->replace_into_registry(m_registry, m_request.snapshot.entities, m_entity_map);
    }

    // Assign current state as the last known remote state which will be used
    // as the initial state for future extrapolations involving these entities.
    // Only necessary to include entities in the snapshot since these are the
    // only ones which changed.
    m_modified_comp->export_remote_state(snapshot_entities);

    // Recalculate properties after setting initial state from server.
    auto origin_view = m_registry.view<position, orientation, center_of_mass, origin>();

    for (auto entity : entities) {
        if (origin_view.contains(entity)) {
            auto [pos, orn, com, orig] = origin_view.get(entity);
            orig = to_world_space(-com, pos, orn);
        }

        if (m_registry.any_of<AABB>(entity)) {
            update_aabb(m_registry, entity);
        }

        if (m_registry.any_of<dynamic_tag>(entity)) {
            update_inertia(m_registry, entity);
        }

        if (m_registry.any_of<rotated_mesh_list>(entity)) {
            update_rotated_mesh(m_registry, entity);
        }
    }

    m_current_entities = std::move(entities);
}

void extrapolation_worker::finish_extrapolation() {
    // Insert modified components into a registry operation to be sent back to
    // the main thread which will assign the extrapolated state to its entities.
    auto &reg_op_ctx = m_registry.ctx().at<registry_operation_context>();
    auto builder = (*reg_op_ctx.make_reg_op_builder)(m_registry);

    // Local entity mapping must not be included if the result is going to be
    // remapped into remote space.
    if (!m_request.should_remap) {
        m_entity_map.each([&](auto remote_entity, auto local_entity) {
            builder->add_entity_mapping(local_entity, remote_entity);
        });
    }

    // Collect owned entities so they can be fed to the export function, which
    // will ignore input components owned by the local client because local
    // inputs must not be overriden by the last value set by extrapolation.
    auto owned_entities = entt::sparse_set{};

    for (auto remote_entity : m_request.owned_entities) {
        auto local_entity = m_entity_map.at(remote_entity);
        owned_entities.emplace(local_entity);
    }

    m_modified_comp->set_observe_changes(false);
    m_modified_comp->export_to_builder(*builder, m_current_entities, owned_entities);
    m_modified_comp->clear_modified(m_current_entities);

    auto result = extrapolation_result{};
    result.ops = std::move(builder->finish());
    EDYN_ASSERT(!result.ops.empty());

    // Insert all manifolds into it. All manifolds in the registry belong to
    // this extrapolation since they're always destroyed at the end of the job.
    auto manifold_view = m_registry.view<contact_manifold>();
    manifold_view.each([&](contact_manifold &manifold) {
        result.manifolds.push_back(manifold);
    });

    // Disable all entities at the end.
    for (auto entity : m_current_entities) {
        m_registry.emplace<disabled_tag>(entity);
    }

    // Assign timestamp of the last step.
    result.timestamp = m_current_time;

    if (m_request.should_remap) {
        m_entity_map.swap();
        result.remap(m_entity_map);
        m_entity_map.swap();
    }

    auto &dispatcher = message_dispatcher::global();
    dispatcher.send<extrapolation_result>(m_request.destination, m_message_queue.identifier, std::move(result));

    // Destroy all contact manifolds to avoid mixing up with future jobs.
    m_registry.destroy(manifold_view.begin(), manifold_view.end());

    m_current_entities.clear();
}

bool extrapolation_worker::should_step() {
    auto time = performance_time();

    if (time - m_init_time > m_request.execution_time_limit) {
        // Timeout.
        m_terminated_early = true;
        return false;
    }

    if (m_current_time > time) {
        // Job is done.
        return false;
    }

    return true;
}

void extrapolation_worker::begin_step() {
    auto &settings = m_registry.ctx().at<edyn::settings>();

    // Clear all action lists before inserting new actions.
    // This will include any actions imported via the registry operations.
    // Very important to clear those to avoid applying fresh new actions right
    // at the beginning of the extrapolation which would lead to large errors.
    if (settings.clear_actions_func) {
        (*settings.clear_actions_func)(m_registry);
    }

    apply_history();

    if (settings.pre_step_callback) {
        (*settings.pre_step_callback)(m_registry);
    }
}

void extrapolation_worker::finish_step() {
    auto &settings = m_registry.ctx().at<edyn::settings>();
    m_current_time += settings.fixed_dt;

    if (settings.post_step_callback) {
        (*settings.post_step_callback)(m_registry);
    }

    ++m_step_count;
}

void extrapolation_worker::extrapolate() {
    init_extrapolation();

    while (should_step()) {
        begin_step();
        m_registry.ctx().at<broadphase>().update(true);
        m_island_manager.update(m_current_time);
        m_registry.ctx().at<narrowphase>().update(true);
        m_solver.update(true);
        finish_step();
    }

    finish_extrapolation();
}

void extrapolation_worker::run() {
    init();

    while (m_running.load(std::memory_order_relaxed)) {
        {
            std::unique_lock<std::mutex> lock(m_mutex);
            m_cv.wait(lock, [&]() {
                return m_has_messages.exchange(false, std::memory_order_relaxed) ||
                    !m_running.load(std::memory_order_relaxed);
            });
        }

        m_message_queue.update();

        if (m_has_work) {
            extrapolate();
            m_has_work = false;
        }
    }

    deinit();
}

}
