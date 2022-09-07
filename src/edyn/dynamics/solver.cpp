#include "edyn/dynamics/solver.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/comp/graph_edge.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/origin.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/dynamics/island_solver.hpp"
#include "edyn/config/execution_mode.hpp"
#include "edyn/dynamics/island_constraint_entities.hpp"
#include "edyn/dynamics/row_cache.hpp"
#include "edyn/parallel/atomic_counter_sync.hpp"
#include "edyn/parallel/job_dispatcher.hpp"
#include "edyn/parallel/parallel_for.hpp"
#include "edyn/parallel/parallel_for_async.hpp"
#include "edyn/serialization/s11n_util.hpp"
#include "edyn/sys/apply_gravity.hpp"
#include "edyn/sys/integrate_spin.hpp"
#include "edyn/sys/update_tire_state.hpp"
#include "edyn/sys/update_aabbs.hpp"
#include "edyn/sys/update_rotated_meshes.hpp"
#include "edyn/sys/update_inertias.hpp"
#include "edyn/sys/update_origins.hpp"
#include "edyn/constraints/constraint_row.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/spin.hpp"
#include "edyn/comp/delta_linvel.hpp"
#include "edyn/comp/delta_angvel.hpp"
#include "edyn/constraints/constraint.hpp"
#include "edyn/util/constraint_util.hpp"
#include "edyn/dynamics/restitution_solver.hpp"
#include "edyn/dynamics/island_solver.hpp"
#include "edyn/context/settings.hpp"
#include "edyn/util/entt_util.hpp"
#include "edyn/util/island_util.hpp"
#include <entt/entity/registry.hpp>
#include <optional>
#include <type_traits>

namespace edyn {

solver::solver(entt::registry &registry)
    : m_registry(&registry)
{
    m_connections.emplace_back(registry.on_construct<linvel>().connect<&entt::registry::emplace<delta_linvel>>());
    m_connections.emplace_back(registry.on_construct<angvel>().connect<&entt::registry::emplace<delta_angvel>>());
    m_connections.emplace_back(registry.on_construct<island_tag>().connect<&entt::registry::emplace<row_cache>>());
    m_connections.emplace_back(registry.on_construct<island_tag>().connect<&entt::registry::emplace<island_constraint_entities>>());
    m_connections.emplace_back(registry.on_construct<constraint_tag>().connect<&entt::registry::emplace<constraint_row_prep_cache>>());
}

template<typename C, typename BodyView, typename OriginView, typename ManifoldView>
void invoke_prepare_constraint(entt::registry &registry, entt::entity entity, C &&con,
                               constraint_row_prep_cache &cache, scalar dt,
                               const BodyView &body_view, const OriginView &origin_view,
                               const ManifoldView &manifold_view) {
    auto [posA, ornA, linvelA, angvelA, inv_mA, inv_IA, dvA, dwA] = body_view.get(con.body[0]);
    auto [posB, ornB, linvelB, angvelB, inv_mB, inv_IB, dvB, dwB] = body_view.get(con.body[1]);

    auto originA = origin_view.contains(con.body[0]) ?
        origin_view.template get<origin>(con.body[0]) : static_cast<vector3>(posA);
    auto originB = origin_view.contains(con.body[1]) ?
        origin_view.template get<origin>(con.body[1]) : static_cast<vector3>(posB);

    cache.add_constraint();

    // Grab index of first row so all rows that will be added can be iterated
    // later to finish their setup. Note that no rows could be added as well.
    auto row_start_index = cache.num_rows;

    if constexpr(std::is_same_v<std::decay_t<C>, contact_constraint>) {
        auto &manifold = manifold_view.template get<contact_manifold>(entity);
        con.prepare(registry, entity, manifold, cache, dt,
                    originA, posA, ornA, linvelA, angvelA, inv_mA, inv_IA,
                    originB, posB, ornB, linvelB, angvelB, inv_mB, inv_IB);
    } else {
        con.prepare(registry, entity, cache, dt,
                    originA, posA, ornA, linvelA, angvelA, inv_mA, inv_IA,
                    originB, posB, ornB, linvelB, angvelB, inv_mB, inv_IB);
    }

    // Assign masses and deltas to new rows.
    for (auto i = row_start_index; i < cache.num_rows; ++i) {
        auto &row = cache.rows[i].row;
        row.inv_mA = inv_mA; row.inv_IA = inv_IA;
        row.inv_mB = inv_mB; row.inv_IB = inv_IB;
        row.dvA = &dvA; row.dwA = &dwA;
        row.dvB = &dvB; row.dwB = &dwB;

        auto &options = cache.rows[i].options;
        prepare_row(row, options, linvelA, angvelA, linvelB, angvelB);
    }
}

static bool prepare_constraints(entt::registry &registry, scalar dt, execution_mode mode,
                                std::optional<job> completion_job = {}) {
    auto body_view = registry.view<position, orientation,
                                   linvel, angvel,
                                   mass_inv, inertia_world_inv,
                                   delta_linvel, delta_angvel>();
    auto origin_view = registry.view<origin>();
    auto cache_view = registry.view<constraint_row_prep_cache>(exclude_sleeping_disabled);
    auto manifold_view = registry.view<contact_manifold>();
    auto con_view_tuple = get_tuple_of_views(registry, constraints_tuple);

    auto for_loop_body = [&registry, body_view, cache_view, origin_view,
                          manifold_view, con_view_tuple, dt](entt::entity entity) {
        auto &prep_cache = cache_view.get<constraint_row_prep_cache>(entity);
        prep_cache.clear();

        std::apply([&](auto &&... con_view) {
            ((con_view.contains(entity) ?
                invoke_prepare_constraint(registry, entity, std::get<0>(con_view.get(entity)), prep_cache,
                                          dt, body_view, origin_view, manifold_view) : void(0)), ...);
        }, con_view_tuple);
    };

    const size_t max_sequential_size = 4;
    auto num_constraints = calculate_view_size(registry.view<constraint_tag>(exclude_sleeping_disabled));

    if (num_constraints <= max_sequential_size || mode == execution_mode::sequential) {
        for (auto entity : cache_view) {
            for_loop_body(entity);
        }
        // Done, return true.
        return true;
    } else if (mode == execution_mode::sequential_multithreaded) {
        auto &dispatcher = job_dispatcher::global();
        parallel_for_each(dispatcher, cache_view.begin(), cache_view.end(), for_loop_body);
        return true;
    } else {
        EDYN_ASSERT(mode == execution_mode::asynchronous);
        auto &dispatcher = job_dispatcher::global();
        parallel_for_each_async(dispatcher, cache_view.begin(), cache_view.end(), *completion_job, for_loop_body);
        // Pending work still running, not done yet, return false.
        return false;
    }
}

bool solver::update(const job &completion_job) {
    auto &registry = *m_registry;
    auto &settings = registry.ctx().at<edyn::settings>();
    auto dt = settings.fixed_dt;

    switch (m_state) {
    case state::begin:
        m_state = state::solve_restitution;
        return update(completion_job);
        break;

    case state::solve_restitution:
        // Apply restitution impulses before gravity to prevent resting objects to
        // start bouncing due to the initial gravity acceleration.
        solve_restitution(registry, dt);
        m_state = state::apply_gravity;
        return update(completion_job);
        break;

    case state::apply_gravity:
        apply_gravity(registry, dt);
        m_state = state::prepare_constraints;
        return update(completion_job);
        break;

    case state::prepare_constraints:
        m_state = state::solve_islands;
        if (prepare_constraints(*m_registry, dt, execution_mode::asynchronous, completion_job)) {
            return update(completion_job);
        } else {
            return false;
        }
        break;

    case state::solve_islands: {
        m_state = state::finalize;

        auto island_view = registry.view<island>(exclude_sleeping_disabled);
        auto num_islands = calculate_view_size(island_view);

        if (num_islands > 0) {
            m_counter = std::make_unique<atomic_counter>(completion_job, num_islands);

            for (auto island_entity : island_view) {
                run_island_solver_async(registry, island_entity,
                                        settings.num_solver_velocity_iterations,
                                        settings.num_solver_position_iterations,
                                        settings.fixed_dt,
                                        m_counter.get());
            }

            return false;
        } else {
            return update(completion_job);
        }
        break;
    }

    case state::finalize:
        update_origins(registry);

        // Update rotated vertices of convex meshes after rotations change. It is
        // important to do this before `update_aabbs` because the rotated meshes
        // will be used to calculate AABBs of polyhedrons.
        update_rotated_meshes(registry);

        // Update AABBs after transforms change.
        update_aabbs(registry);
        update_island_aabbs(registry);

        // Update world-space moment of inertia.
        update_inertias(registry);
        m_state = state::done;
        return update(completion_job);
        break;

    case state::done:
        m_state = state::begin;
        return true;
    }
}

void solver::update_sequential(bool mt) {
    auto &registry = *m_registry;
    auto island_view = registry.view<island>(exclude_sleeping_disabled);
    auto num_islands = calculate_view_size(island_view);

    if (num_islands == 0) {
        return;
    }

    auto &settings = registry.ctx().at<edyn::settings>();
    auto dt = settings.fixed_dt;

    solve_restitution(registry, dt);
    apply_gravity(registry, dt);

    auto exec_mode = mt ? execution_mode::sequential_multithreaded : execution_mode::sequential;
    prepare_constraints(registry, dt, exec_mode);

    if (mt && num_islands > 1) {
        auto counter = atomic_counter_sync(num_islands);

        for (auto island_entity : island_view) {
            run_island_solver_seq_mt(registry, island_entity,
                                     settings.num_solver_velocity_iterations,
                                     settings.num_solver_position_iterations,
                                     settings.fixed_dt,
                                     &counter);
        }

        counter.wait();
    } else {
        for (auto island_entity : island_view) {
            run_island_solver_seq(registry, island_entity,
                                  settings.num_solver_velocity_iterations,
                                  settings.num_solver_position_iterations,
                                  settings.fixed_dt);
        }

    case state::finalize:
        update_origins(registry);

        // Update rotated vertices of convex meshes after rotations change. It is
        // important to do this before `update_aabbs` because the rotated meshes
        // will be used to calculate AABBs of polyhedrons.
        update_rotated_meshes(registry);

        // Update AABBs after transforms change.
        update_aabbs(registry);
        update_island_aabbs(registry);

        // Update world-space moment of inertia.
        update_inertias(registry);
        m_state = state::done;
        return update(completion_job);
        break;

    case state::done:
        m_state = state::begin;
        return true;
    }
}

void solver::update_sequential(bool mt) {
    auto &registry = *m_registry;
    auto island_view = registry.view<island>(exclude_sleeping_disabled);
    auto num_islands = calculate_view_size(island_view);

    if (num_islands == 0) {
        return;
    }

    auto &settings = registry.ctx().at<edyn::settings>();
    auto dt = settings.fixed_dt;

    solve_restitution(registry, dt);
    apply_gravity(registry, dt);

    auto exec_mode = mt ? execution_mode::sequential_multithreaded : execution_mode::sequential;
    prepare_constraints(registry, dt, exec_mode);

    if (mt && num_islands > 1) {
        auto counter = atomic_counter_sync(num_islands);

        for (auto island_entity : island_view) {
            run_island_solver_seq_mt(registry, island_entity,
                                     settings.num_solver_velocity_iterations,
                                     settings.num_solver_position_iterations,
                                     settings.fixed_dt,
                                     &counter);
        }

        counter.wait();
    } else {
        for (auto island_entity : island_view) {
            run_island_solver_seq(registry, island_entity,
                                  settings.num_solver_velocity_iterations,
                                  settings.num_solver_position_iterations,
                                  settings.fixed_dt);
        }
    }

    update_origins(registry);

    // Update rotated vertices of convex meshes after rotations change. It is
    // important to do this before `update_aabbs` because the rotated meshes
    // will be used to calculate AABBs of polyhedrons.
    update_rotated_meshes(registry);

    // Update AABBs after transforms change.
    update_aabbs(registry);
    update_island_aabbs(registry);

    // Update world-space moment of inertia.
    update_inertias(registry);
}

}
