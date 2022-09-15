#include "edyn/networking/extrapolation/extrapolation_job.hpp"
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
#include "edyn/networking/util/input_state_history.hpp"
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
#include <atomic>
#include <entt/entity/fwd.hpp>

namespace edyn {

void extrapolation_job_func(job::data_type &data) {
    auto archive = memory_input_archive(data.data(), data.size());
    intptr_t job_intptr;
    archive(job_intptr);
    auto *job = reinterpret_cast<extrapolation_job *>(job_intptr);
    job->update();
}

extrapolation_job::extrapolation_job(extrapolation_input &&input,
                                     const settings &settings,
                                     const registry_operation_context &reg_op_ctx,
                                     const material_mix_table &material_table,
                                     std::shared_ptr<input_state_history> input_history,
                                     make_extrapolation_modified_comp_func_t *make_extrapolation_modified_comp)
    : m_input(std::move(input))
    , m_state(state::init)
    , m_current_time(input.start_time)
    , m_solver(m_registry)
    , m_input_history(input_history)
    , m_poly_initializer(m_registry)
    , m_island_manager(m_registry)
    , m_make_extrapolation_modified_comp(make_extrapolation_modified_comp)
{
    m_registry.ctx().emplace<broadphase>(m_registry);
    m_registry.ctx().emplace<narrowphase>(m_registry);
    m_registry.ctx().emplace<entity_graph>();
    m_registry.ctx().emplace<edyn::settings>(settings);
    m_registry.ctx().emplace<registry_operation_context>(reg_op_ctx);
    m_registry.ctx().emplace<contact_manifold_map>(m_registry);
    m_registry.ctx().emplace<material_mix_table>(material_table);

    m_this_job.func = &extrapolation_job_func;
    auto archive = fixed_memory_output_archive(m_this_job.data.data(), m_this_job.data.size());
    auto ctx_intptr = reinterpret_cast<intptr_t>(this);
    archive(ctx_intptr);
}

void extrapolation_job::load_input() {
    // The registry is expected to be empty before importing the input.
    EDYN_ASSERT(m_registry.empty());

    // Import entities and components.
    m_input.ops.execute(m_registry, m_entity_map);

    auto &graph = m_registry.ctx().at<entity_graph>();
    auto node_view = m_registry.view<graph_node>();
    auto procedural_view = m_registry.view<procedural_tag>();

    // Create nodes for rigid bodies in entity graph.
    auto insert_graph_node = [&](entt::entity entity) {
        auto procedural = procedural_view.contains(entity);
        auto node_index = graph.insert_node(entity, !procedural);
        m_registry.emplace<graph_node>(entity, node_index);
    };

    std::apply([&](auto ... t) {
        (m_registry.view<decltype(t)>().each(insert_graph_node), ...);
    }, std::tuple<rigidbody_tag, external_tag>{});

    // Create edges for constraints in entity graph.
    auto insert_graph_edge = [&](entt::entity entity, auto &&con) {
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
    m_island_manager.update(m_start_time);

    // Initialize shapes.
    m_poly_initializer.init_new_shapes();

    // Replace client component state by server state.
    for (auto &pool : m_input.snapshot.pools) {
        pool.ptr->replace_into_registry(m_registry, m_input.snapshot.entities, m_entity_map);
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

    for (auto remote_entity : m_input.entities) {
        auto local_entity = m_entity_map.at(remote_entity);
        relevant_entities.emplace(local_entity);
    }

    for (auto remote_entity : m_input.owned_entities) {
        auto local_entity = m_entity_map.at(remote_entity);
        owned_entities.emplace(local_entity);
    }

    m_modified_comp = (*m_make_extrapolation_modified_comp)(m_registry, relevant_entities, owned_entities);
}

void extrapolation_job::init() {
    m_start_time = performance_time();
    m_island_manager.set_last_time(m_start_time);

    // Import entities and components to be extrapolated.
    load_input();

    m_state = state::step;
}

void extrapolation_job::apply_history() {
    auto &settings = m_registry.ctx().at<edyn::settings>();
    auto start_time = m_current_time - settings.fixed_dt;
    m_input_history->import_each(start_time, settings.fixed_dt, m_registry, m_entity_map);
}

void extrapolation_job::sync_and_finish() {

    // Insert modified components into a registry operation to be sent back to
    // the main thread which will assign the extrapolated state to its entities.
    auto &reg_op_ctx = m_registry.ctx().at<registry_operation_context>();
    auto builder = (*reg_op_ctx.make_reg_op_builder)(m_registry);

    // Local entity mapping must not be included if the result is going to be
    // remapped into remote space.
    if (!m_input.should_remap) {
        m_entity_map.each([&](auto remote_entity, auto local_entity) {
            builder->add_entity_mapping(local_entity, remote_entity);
        });
    }

    m_modified_comp->export_to_builder(*builder);

    auto body_view = m_registry.view<position, orientation, linvel, angvel>();
    builder->replace<position>(body_view.begin(), body_view.end());
    builder->replace<orientation>(body_view.begin(), body_view.end());
    builder->replace<linvel>(body_view.begin(), body_view.end());
    builder->replace<angvel>(body_view.begin(), body_view.end());

    m_result.ops = std::move(builder->finish());
    EDYN_ASSERT(!m_result.ops.empty());

    // Insert all manifolds into it.
    auto manifold_view = m_registry.view<contact_manifold>();
    manifold_view.each([&](contact_manifold &manifold) {
        m_result.manifolds.push_back(manifold);
    });

    // Assign timestamp of the last step.
    m_result.timestamp = m_current_time;

    if (m_input.should_remap) {
        m_entity_map.swap();
        m_result.remap(m_entity_map);
    }

    m_finished.store(true, std::memory_order_release);
}

void extrapolation_job::run_state_machine() {
    switch (m_state) {
    case state::init:
        init();
        m_state = state::step;
        run_state_machine();
        break;
    case state::step:
        if (should_step()) {
            m_state = state::begin_step;
        } else {
            m_state = state::done;
        }
        run_state_machine();
        break;
    case state::begin_step:
        begin_step();
        m_state = state::broadphase;
        run_state_machine();
        break;
    case state::broadphase:
        if (m_registry.ctx().at<broadphase>().update_async(m_this_job)) {
            // Broadphase creates and destroys manifolds, which are edges in
            // the entity graph. Thus, it is necessary to initialize new edges
            // and split islands right after.
            m_state = state::update_islands;
            run_state_machine();
        }
        break;
    case state::update_islands:
        m_island_manager.update(m_current_time);
        m_state = state::narrowphase;
        run_state_machine();
        break;
    case state::narrowphase:
        if (m_registry.ctx().at<narrowphase>().update_async(m_this_job)) {
            m_state = state::solve;
            run_state_machine();
        }
        break;
    case state::solve:
        if (m_solver.update_async(m_this_job)) {
            m_state = state::finish_step;
            run_state_machine();
        }
        break;
    case state::finish_step:
        finish_step();
        m_state = state::step;
        reschedule();
        break;
    case state::done:
        sync_and_finish();
        break;
    }
}

void extrapolation_job::update() {
    run_state_machine();
}

bool extrapolation_job::should_step() {
    auto time = performance_time();

    if (time - m_start_time > m_input.execution_time_limit) {
        // Timeout.
        m_result.terminated_early = true;
        return false;
    }

    auto &settings = m_registry.ctx().at<edyn::settings>();

    if (m_current_time + settings.fixed_dt > time) {
        // Job is done.
        return false;
    }

    return true;
}

void extrapolation_job::begin_step() {
    EDYN_ASSERT(m_state == state::begin_step);

    apply_history();

    auto &settings = m_registry.ctx().at<edyn::settings>();
    if (settings.pre_step_callback) {
        (*settings.pre_step_callback)(m_registry);
    }

    m_state = state::broadphase;
}

void extrapolation_job::finish_step() {
    EDYN_ASSERT(m_state == state::finish_step);

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
    m_state = state::step;
}

void extrapolation_job::reschedule() {
    job_dispatcher::global().async(m_this_job);
}

}
