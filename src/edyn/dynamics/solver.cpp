#include "edyn/dynamics/solver.hpp"
#include "edyn/comp/graph_edge.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/origin.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/dynamics/island_solver.hpp"
#include "edyn/dynamics/row_cache.hpp"
#include "edyn/parallel/atomic_counter_sync.hpp"
#include "edyn/parallel/job_dispatcher.hpp"
#include "edyn/parallel/parallel_for.hpp"
#include "edyn/parallel/parallel_for_async.hpp"
#include "edyn/serialization/s11n_util.hpp"
#include "edyn/sys/apply_gravity.hpp"
#include "edyn/sys/update_aabbs.hpp"
#include "edyn/sys/update_rotated_meshes.hpp"
#include "edyn/sys/update_inertias.hpp"
#include "edyn/sys/update_origins.hpp"
#include "edyn/constraints/constraint_row.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/delta_linvel.hpp"
#include "edyn/comp/delta_angvel.hpp"
#include "edyn/constraints/constraint.hpp"
#include "edyn/util/constraint_util.hpp"
#include "edyn/dynamics/restitution_solver.hpp"
#include "edyn/dynamics/island_solver.hpp"
#include "edyn/context/settings.hpp"
#include "edyn/util/island_util.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

template<typename View>
size_t view_size(const View &view) {
    return std::distance(view.begin(), view.end());
}

solver::solver(entt::registry &registry)
    : m_registry(&registry)
{
    registry.on_construct<linvel>().connect<&entt::registry::emplace<delta_linvel>>();
    registry.on_construct<angvel>().connect<&entt::registry::emplace<delta_angvel>>();
    registry.on_construct<island_tag>().connect<&entt::registry::emplace<row_cache>>();
    registry.on_construct<constraint_tag>().connect<&entt::registry::emplace<constraint_row_prep_cache>>();
}

template<typename C>
void prepare_constraints_seq(entt::registry &registry, scalar dt, bool mt) {
    auto body_view = registry.view<position, orientation,
                                   linvel, angvel,
                                   mass_inv, inertia_world_inv,
                                   delta_linvel, delta_angvel>();
    auto origin_view = registry.view<origin>();
    auto con_view = registry.view<C, constraint_row_prep_cache>(exclude_sleeping_disabled);

    auto for_loop_body = [&registry, body_view, con_view, origin_view, dt](entt::entity entity, size_t index) {
        auto [con, cache] = con_view.get(entity);
        auto [posA, ornA, linvelA, angvelA, inv_mA, inv_IA, dvA, dwA] = body_view.get(con.body[0]);
        auto [posB, ornB, linvelB, angvelB, inv_mB, inv_IB, dvB, dwB] = body_view.get(con.body[1]);

        auto originA = origin_view.contains(con.body[0]) ? origin_view.get<origin>(con.body[0]) : static_cast<vector3>(posA);
        auto originB = origin_view.contains(con.body[1]) ? origin_view.get<origin>(con.body[1]) : static_cast<vector3>(posB);

        cache.count = 0;

        prepare_constraint(registry, entity, con, cache, dt,
                           originA, posA, ornA, linvelA, angvelA, inv_mA, inv_IA, dvA, dwA,
                           originB, posB, ornB, linvelB, angvelB, inv_mB, inv_IB, dvB, dwB);
    };

    if (mt && view_size(con_view) > 4) {
        auto &dispatcher = job_dispatcher::global();
        parallel_for_each(dispatcher, con_view.begin(), con_view.end(), for_loop_body);
    } else {
        size_t index = 0;
        for (auto entity : con_view) {
            for_loop_body(entity, index++);
        }
    }
}

template<typename C>
void prepare_constraints_async(entt::registry &registry, scalar dt, const job &decr_job) {
    auto body_view = registry.view<position, orientation,
                                   linvel, angvel,
                                   mass_inv, inertia_world_inv,
                                   delta_linvel, delta_angvel>();
    auto origin_view = registry.view<origin>();
    auto con_view = registry.view<C, constraint_row_prep_cache>(exclude_sleeping_disabled);
    auto &dispatcher = job_dispatcher::global();

    parallel_for_each_async(dispatcher, con_view.begin(), con_view.end(), decr_job,
        [&registry, body_view, con_view, origin_view, dt](entt::entity entity, size_t index) {
            auto [con, cache] = con_view.get(entity);
            auto [posA, ornA, linvelA, angvelA, inv_mA, inv_IA, dvA, dwA] = body_view.get(con.body[0]);
            auto [posB, ornB, linvelB, angvelB, inv_mB, inv_IB, dvB, dwB] = body_view.get(con.body[1]);

            auto originA = origin_view.contains(con.body[0]) ? origin_view.get<origin>(con.body[0]) : static_cast<vector3>(posA);
            auto originB = origin_view.contains(con.body[1]) ? origin_view.get<origin>(con.body[1]) : static_cast<vector3>(posB);

            cache.count = 0;

            prepare_constraint(registry, entity, con, cache, dt,
                               originA, posA, ornA, linvelA, angvelA, inv_mA, inv_IA, dvA, dwA,
                               originB, posB, ornB, linvelB, angvelB, inv_mB, inv_IB, dvB, dwB);
        });
}

template<>
void prepare_constraints_seq<null_constraint>(entt::registry &, scalar, bool) {}

template<>
void prepare_constraints_async<null_constraint>(entt::registry &, scalar, const job &) {
    EDYN_ASSERT(false);
}

bool solver::prepare_constraints(const job &completion_job, scalar dt) {
    std::array<size_t,std::tuple_size_v<constraints_tuple_t>> sizes;
    std::apply([&](auto ... c) {
        size_t idx = 0;
        ((sizes[idx++] =
            std::is_same_v<decltype(c), null_constraint> ? size_t(0) :
            view_size(m_registry->view<decltype(c)>(exclude_sleeping_disabled))), ...);
    }, constraints_tuple);

    unsigned num_async = 0;
    const size_t max_sequential_size = 2;

    // Prepare the sequential ones first.
    std::apply([&](auto ... c) {
        size_t idx = 0;
        (((sizes[idx] <= max_sequential_size ?
            prepare_constraints_seq<decltype(c)>(*m_registry, dt, false) : (void)(++num_async)), ++idx), ...);
    }, constraints_tuple);

    // Prepare the parallel ones later.
    if (num_async > 0) {
        // Multiple asynchronous parallel-for loops will be started. When they
        // finish, they decrement this atomic counter which, when it reaches zero,
        // will dispatch the `completion_job` to run again, which will invoke
        // this function so we can continue in the `pack_rows` state.
        m_counter = std::make_unique<atomic_counter>(completion_job, num_async);
        auto decrement_job = m_counter->get_decrement_job();

        std::apply([&](auto ... c) {
            size_t idx = 0;
            (((sizes[idx] > max_sequential_size ?
                prepare_constraints_async<decltype(c)>(*m_registry, dt, decrement_job) : (void)0), ++idx), ...);
        }, constraints_tuple);

        return false;
    }

    return true;
}

void solver::prepare_constraints_sequential(bool mt, scalar dt) {
    std::apply([&](auto ... c) {
        size_t idx = 0;
        ((prepare_constraints_seq<decltype(c)>(*m_registry, dt, mt), ++idx), ...);
    }, constraints_tuple);
}

static void apply_solution(entt::registry &registry, scalar dt) {
    auto view = registry.view<position, orientation,
                              linvel, angvel, delta_linvel,
                              delta_angvel, dynamic_tag>(exclude_sleeping_disabled);
    for (auto [e, pos, orn, v, w, dv, dw] : view.each()) {
        // Apply deltas.
        v += dv;
        w += dw;
        // Integrate velocities and obtain new transforms.
        pos += v * dt;
        orn = integrate(orn, w, dt);
        // Reset deltas back to zero for next update.
        dv = vector3_zero;
        dw = vector3_zero;
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
        m_state = state::solve_constraints;
        if (prepare_constraints(completion_job, dt)) {
            return update(completion_job);
        } else {
            return false;
        }
        break;

    case state::solve_constraints: {
        m_state = state::apply_solution;
        auto island_view = registry.view<island>(exclude_sleeping_disabled);
        auto num_islands = view_size(island_view);

        if (num_islands > 0) {
            m_counter = std::make_unique<atomic_counter>(completion_job, num_islands);

            for (auto island_entity : island_view) {
                run_island_solver_async(registry, island_entity,
                                        settings.num_solver_velocity_iterations,
                                        m_counter.get());
            }

            return false;
        } else {
            return update(completion_job);
        }
        break;
    }
    case state::apply_solution:
        // Apply constraint velocity correction.
        apply_solution(registry, dt);
        m_state = state::solve_position_constraints;
        return update(completion_job);
        break;

    case state::solve_position_constraints:
        // Now that rigid bodies have moved, perform positional correction.
        for (unsigned i = 0; i < settings.num_solver_position_iterations; ++i) {
            if (solve_position_constraints(registry, dt)) {
                break;
            }
        }
        m_state = state::finalize;
        return update(completion_job);
        break;

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
    auto num_islands = view_size(island_view);

    if (num_islands == 0) {
        return;
    }

    auto &settings = registry.ctx().at<edyn::settings>();
    auto dt = settings.fixed_dt;

    solve_restitution(registry, dt);
    apply_gravity(registry, dt);
    prepare_constraints_sequential(mt, dt);

    if (mt && num_islands > 1) {
        auto counter = atomic_counter_sync(num_islands);

        for (auto island_entity : island_view) {
            run_island_solver_seq_mt(registry, island_entity, settings.num_solver_velocity_iterations, &counter);
        }

        counter.wait();
    } else {
        for (auto island_entity : island_view) {
            run_island_solver_seq(registry, island_entity, settings.num_solver_velocity_iterations);
        }
    }

    // Apply constraint velocity correction.
    apply_solution(registry, dt);

    // Now that rigid bodies have moved, perform positional correction.
    for (unsigned i = 0; i < settings.num_solver_position_iterations; ++i) {
        if (solve_position_constraints(registry, dt)) {
            break;
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
