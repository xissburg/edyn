#include "edyn/networking/extrapolation_job.hpp"
#include "edyn/collision/broadphase.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/collision/narrowphase.hpp"
#include "edyn/collision/contact_manifold_map.hpp"
#include "edyn/comp/dirty.hpp"
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
#include "edyn/replication/component_index_source.hpp"
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

namespace edyn {

extern bool(*g_is_networked_input_component)(entt::id_type);
extern bool(*g_is_action_list_component)(entt::id_type);

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
                                     std::shared_ptr<input_state_history> input_history)
    : m_input(std::move(input))
    , m_state(state::init)
    , m_current_time(input.start_time)
    , m_solver(m_registry)
    , m_input_history(input_history)
    , m_poly_initializer(m_registry)
    , m_island_manager(m_registry)
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
}

void extrapolation_job::init() {
    m_start_time = performance_time();
    m_island_manager.set_last_time(m_start_time);

    // Import entities and components to be extrapolated.
    load_input();

    // Initialize external systems.
    auto &settings = m_registry.ctx().at<edyn::settings>();
    if (settings.external_system_init) {
        (*settings.external_system_init)(m_registry);
    }

    m_state = state::step;
}

void extrapolation_job::apply_history() {
    auto &settings = m_registry.ctx().at<edyn::settings>();
    auto start_time = m_current_time - settings.fixed_dt;
    m_input_history->import_each(start_time, settings.fixed_dt, m_registry, m_entity_map);
}

void extrapolation_job::sync_and_finish() {
    // Update continuous components.
    auto &settings = m_registry.ctx().at<edyn::settings>();
    auto &index_source = *settings.index_source;
    auto manifold_view = m_registry.view<contact_manifold>();

    // Collect entities per type to be updated, including only components that
    // have changed, i.e. continuous and dirty components.
    auto &reg_op_ctx = m_registry.ctx().at<registry_operation_context>();
    auto builder = (*reg_op_ctx.make_reg_op_builder)(m_registry);

    // Local entity mapping must not be included if the result is going to be
    // remapped into remote space.
    if (!m_input.should_remap) {
        m_entity_map.each([&](auto remote_entity, auto local_entity) {
            builder->add_entity_mapping(local_entity, remote_entity);
        });
    }

    for (auto remote_entity : m_input.entities) {
        if (!m_entity_map.contains(remote_entity)) continue;

        auto local_entity = m_entity_map.at(remote_entity);

        // Manifolds are shared separately.
        if (!m_registry.valid(local_entity) || manifold_view.contains(local_entity)) continue;

        // Do not include input components of entities owned by the client,
        // since that would cause the latest user inputs to be replaced.
        // Note the use of `remote_entity` next.
        auto is_owned_entity = m_input.owned_entities.contains(remote_entity);

        if (auto *cont = m_registry.try_get<continuous>(local_entity)) {
            for (size_t i = 0; i < cont->size; ++i) {
                auto id = index_source.type_id_of(cont->indices[i]);

                if (!is_owned_entity ||
                    !((*g_is_networked_input_component)(id) || (*g_is_action_list_component)(id))) {
                    builder->replace_type_id(local_entity, id);
                }
            }
        }

        if (auto *dirty = m_registry.try_get<edyn::dirty>(local_entity)) {
            // Only consider updated indices. Entities and components shouldn't be
            // created during extrapolation.
            for (auto id : dirty->updated_ids) {
                if (!is_owned_entity ||
                    !((*g_is_networked_input_component)(id) || (*g_is_action_list_component)(id))) {
                    builder->replace_type_id(local_entity, id);
                }
            }
        }

        m_result.entities.push_back(local_entity);
    }

    m_result.ops = std::move(builder->finish());
    EDYN_ASSERT(!m_result.ops.empty());

    // Insert all manifolds into it.
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
        if (m_registry.ctx().at<broadphase>().update(m_this_job)) {
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
        if (m_registry.ctx().at<narrowphase>().update(m_this_job)) {
            m_state = state::solve;
            run_state_machine();
        }
        break;
    case state::solve:
        if (m_solver.update(m_this_job)) {
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
    if (settings.external_system_pre_step) {
        (*settings.external_system_pre_step)(m_registry);
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

    if (settings.external_system_post_step) {
        (*settings.external_system_post_step)(m_registry);
    }

    ++m_step_count;
    m_state = state::step;
}

void extrapolation_job::reschedule() {
    job_dispatcher::global().async(m_this_job);
}

}
