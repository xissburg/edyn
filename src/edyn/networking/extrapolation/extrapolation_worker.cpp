#include "edyn/networking/extrapolation/extrapolation_worker.hpp"
#include "edyn/collision/broadphase.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/collision/narrowphase.hpp"
#include "edyn/collision/contact_manifold_map.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/rotated_mesh_list.hpp"
#include "edyn/config/config.h"
#include "edyn/constraints/contact_constraint.hpp"
#include "edyn/context/registry_operation_context.hpp"
#include "edyn/context/settings.hpp"
#include "edyn/dynamics/material_mixing.hpp"
#include "edyn/networking/context/client_network_context.hpp"
#include "edyn/networking/extrapolation/extrapolation_request.hpp"
#include "edyn/networking/util/input_state_history.hpp"
#include "edyn/parallel/message.hpp"
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
#include <atomic>

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
    , m_make_extrapolation_modified_comp(make_extrapolation_modified_comp)
    , m_message_queue(message_dispatcher::global().make_queue<
        extrapolation_request,
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
    m_message_queue.sink<msg::set_settings>().connect<&extrapolation_worker::on_set_settings>(*this);
    m_message_queue.sink<msg::set_registry_operation_context>().connect<&extrapolation_worker::on_set_reg_op_ctx>(*this);
    m_message_queue.sink<msg::set_material_table>().connect<&extrapolation_worker::on_set_material_table>(*this);
    m_message_queue.sink<msg::set_extrapolator_context_settings>().connect<&extrapolation_worker::on_set_extrapolator_context_settings>(*this);
    m_message_queue.push_sink().connect<&extrapolation_worker::on_push_message>(*this);
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
    m_destination_queue = msg.sender;
    m_request = std::move(msg.content);
    m_has_work = true;
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
    m_make_extrapolation_modified_comp = msg.content.make_extrapolation_modified_comp;
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

    // Import entities and components.
    m_request.ops.execute(m_registry, m_entity_map);

    auto &graph = m_registry.ctx().at<entity_graph>();
    auto node_view = m_registry.view<graph_node>();
    auto procedural_view = m_registry.view<procedural_tag>();

    // Create nodes for rigid bodies in entity graph.
    auto insert_graph_node = [&](entt::entity entity) {
        auto procedural = procedural_view.contains(entity);
        auto node_index = graph.insert_node(entity, !procedural);
        m_registry.emplace<graph_node>(entity, node_index);

        if (!procedural) {
            // `multi_island_resident` is not a shared component thus add it
            // manually here.
            m_registry.emplace<multi_island_resident>(entity);
        }
    };

    std::apply([&](auto ... t) {
        (m_registry.view<decltype(t)>().each(insert_graph_node), ...);
    }, std::tuple<rigidbody_tag, external_tag>{});

    // Create edges for constraints in entity graph.
    auto insert_graph_edge = [&](entt::entity entity, auto &&con) {
        if (m_registry.all_of<graph_edge>(entity)) {
            return;
        }

        auto &node0 = node_view.get<graph_node>(con.body[0]);
        auto &node1 = node_view.get<graph_node>(con.body[1]);
        auto edge_index = graph.insert_edge(entity, node0.node_index, node1.node_index);
        m_registry.emplace<graph_edge>(entity, edge_index);
    };

    std::apply([&](auto ... t) {
        (m_registry.view<decltype(t)>().each(insert_graph_edge), ...);
    }, constraints_tuple);

    m_registry.view<null_constraint>().each(insert_graph_edge);

    // Create islands.
    m_island_manager.update(m_current_time);

    // Initialize shapes.
    m_poly_initializer.init_new_shapes();

    // Replace client component state by server state.
    for (auto &pool : m_request.snapshot.pools) {
        pool.ptr->replace_into_registry(m_registry, m_request.snapshot.entities, m_entity_map);
    }

    // Apply all inputs before the current time to start the simulation
    // with the correct initial inputs.
    m_input_history->import_initial_state(m_registry, m_entity_map, m_current_time);

    // Recalculate properties after setting initial state from server.
    update_origins(m_registry);
    update_rotated_meshes(m_registry);
    update_aabbs(m_registry);
    update_inertias(m_registry);

    auto relevant_entities = entt::sparse_set{};
    auto owned_entities = entt::sparse_set{};

    for (auto remote_entity : m_request.entities) {
        auto local_entity = m_entity_map.at(remote_entity);
        relevant_entities.emplace(local_entity);
    }

    for (auto remote_entity : m_request.owned_entities) {
        auto local_entity = m_entity_map.at(remote_entity);
        owned_entities.emplace(local_entity);
    }

    m_modified_comp = (*m_make_extrapolation_modified_comp)(m_registry, relevant_entities, owned_entities);

    // All components present in the remote snapshot must be marked as modified
    // because if they do not change during extrapolation, their state won't be
    // included in the final result, since only components that changed are
    // reported back (via `m_modified_comp` which monitors and records changes)
    // to avoid having to include everything. This would lead to the latest
    // remote state of these components not being applied into the local simulation.
    m_modified_comp->mark_snapshot(m_registry, m_request.snapshot, m_entity_map);
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

    m_modified_comp->export_to_builder(*builder);

    auto body_view = m_registry.view<position, orientation, linvel, angvel, dynamic_tag>();
    builder->replace<position>(body_view.begin(), body_view.end());
    builder->replace<orientation>(body_view.begin(), body_view.end());
    builder->replace<linvel>(body_view.begin(), body_view.end());
    builder->replace<angvel>(body_view.begin(), body_view.end());

    auto result = extrapolation_result{};
    result.ops = std::move(builder->finish());
    EDYN_ASSERT(!result.ops.empty());

    // Insert all manifolds into it.
    auto manifold_view = m_registry.view<contact_manifold>();
    manifold_view.each([&](contact_manifold &manifold) {
        result.manifolds.push_back(manifold);
    });

    // Assign timestamp of the last step.
    result.timestamp = m_current_time;

    if (m_request.should_remap) {
        m_entity_map.swap();
        result.remap(m_entity_map);
    }

    auto &dispatcher = message_dispatcher::global();
    dispatcher.send<extrapolation_result>(m_destination_queue, m_message_queue.identifier, std::move(result));

    m_registry.clear();
    m_entity_map.clear();
}

bool extrapolation_worker::should_step() {
    auto time = performance_time();

    if (time - m_init_time > m_request.execution_time_limit) {
        // Timeout.
        m_terminated_early = true;
        return false;
    }

    auto &settings = m_registry.ctx().at<edyn::settings>();

    if (m_current_time + settings.fixed_dt > time) {
        // Job is done.
        return false;
    }

    return true;
}

void extrapolation_worker::begin_step() {
    apply_history();

    auto &settings = m_registry.ctx().at<edyn::settings>();
    if (settings.pre_step_callback) {
        (*settings.pre_step_callback)(m_registry);
    }
}

void extrapolation_worker::finish_step() {
    auto &settings = m_registry.ctx().at<edyn::settings>();
    m_current_time += settings.fixed_dt;

     // Clear actions after they've been consumed.
    if (settings.clear_actions_func) {
        (*settings.clear_actions_func)(m_registry);
    }

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
