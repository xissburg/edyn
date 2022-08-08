#include "edyn/dynamics/solver.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/origin.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/constraints/soft_distance_constraint.hpp"
#include "edyn/dynamics/row_cache.hpp"
#include "edyn/parallel/job_dispatcher.hpp"
#include "edyn/parallel/parallel_for.hpp"
#include "edyn/parallel/parallel_for_async.hpp"
#include "edyn/sys/apply_gravity.hpp"
#include "edyn/sys/integrate_linvel.hpp"
#include "edyn/sys/integrate_angvel.hpp"
#include "edyn/sys/update_aabbs.hpp"
#include "edyn/sys/update_rotated_meshes.hpp"
#include "edyn/sys/update_inertias.hpp"
#include "edyn/sys/update_origins.hpp"
#include "edyn/constraints/constraint_row.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/delta_linvel.hpp"
#include "edyn/comp/delta_angvel.hpp"
#include "edyn/collision/contact_point.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/constraints/constraint.hpp"
#include "edyn/util/constraint_util.hpp"
#include "edyn/dynamics/restitution_solver.hpp"
#include "edyn/context/settings.hpp"
#include <entt/entity/registry.hpp>
#include <tuple>
#include <type_traits>

namespace edyn {

template<typename View>
size_t view_size(const View &view) {
    return std::distance(view.begin(), view.end());
}

scalar solve(constraint_row &row) {
    auto delta_relvel = dot(row.J[0], *row.dvA) +
                        dot(row.J[1], *row.dwA) +
                        dot(row.J[2], *row.dvB) +
                        dot(row.J[3], *row.dwB);
    auto delta_impulse = (row.rhs - delta_relvel) * row.eff_mass;
    auto impulse = row.impulse + delta_impulse;

    if (impulse < row.lower_limit) {
        delta_impulse = row.lower_limit - row.impulse;
        row.impulse = row.lower_limit;
    } else if (impulse > row.upper_limit) {
        delta_impulse = row.upper_limit - row.impulse;
        row.impulse = row.upper_limit;
    } else {
        row.impulse = impulse;
    }

    return delta_impulse;
}

template<typename C>
void update_impulse(entt::registry &registry, row_cache &cache, size_t &con_idx, size_t &row_idx) {
    auto con_view = registry.view<C>(entt::exclude_t<disabled_tag, sleeping_tag>{});

    for (auto entity : con_view) {
        auto [con] = con_view.get(entity);
        auto num_rows = cache.con_num_rows[con_idx];

        // Check if the constraint `impulse` member is a scalar, otherwise
        // an array is expected.
        if constexpr(std::is_same_v<decltype(con.impulse), scalar>) {
            EDYN_ASSERT(num_rows == 1);
            con.impulse = cache.rows[row_idx].impulse;
        } else {
            EDYN_ASSERT(con.impulse.size() >= num_rows);
            for (size_t i = 0; i < num_rows; ++i) {
                con.impulse[i] = cache.rows[row_idx + i].impulse;
            }
        }

        row_idx += num_rows;
        ++con_idx;
    }
}

// Specialization for `null_constraint` which is a no-op.
template<>
void update_impulse<null_constraint>(entt::registry &, row_cache &, size_t &, size_t &) {}

// Specialization to assign the impulses of friction constraints which are not
// stored in traditional constraint rows.
/* template<>
void update_impulse<contact_constraint>(entt::registry &registry, row_cache &cache, size_t &con_idx, size_t &row_idx) {
    auto con_view = registry.view<contact_constraint, contact_manifold>(entt::exclude_t<disabled_tag, sleeping_tag>{});
    auto &ctx = registry.ctx().at<internal::contact_constraint_context>();
    auto global_pt_idx = size_t(0);
    auto roll_idx = size_t(0);

    for (auto entity : con_view) {
        auto &manifold = con_view.get<contact_manifold>(entity);

        for (unsigned pt_idx = 0; pt_idx < manifold.num_points; ++pt_idx) {
            auto &cp = manifold.get_point(pt_idx);
            cp.normal_impulse = cache.rows[row_idx++].impulse;

            // Friction impulse.
            auto &friction_rows = ctx.friction_rows[global_pt_idx];

            for (auto i = 0; i < 2; ++i) {
                cp.friction_impulse[i] = friction_rows.row[i].impulse;
            }

            // Rolling friction impulse.
            if (cp.roll_friction > 0) {
                auto &roll_rows = ctx.roll_friction_rows[roll_idx];
                for (auto i = 0; i < 2; ++i) {
                    cp.rolling_friction_impulse[i] = roll_rows.row[i].impulse;
                }
                ++roll_idx;
            }

            // Spinning friction impulse.
            if (cp.spin_friction > 0) {
                cp.spin_friction_impulse = cache.rows[row_idx++].impulse;
            }

            ++global_pt_idx;
        }

        ++con_idx;
    }
} */

void update_impulses(entt::registry &registry, row_cache &cache) {
    // Assign impulses from constraint rows back into the constraints. The rows
    // are inserted into the cache for each constraint type in the order they're
    // found in `constraints_tuple` and in the same order they're in their EnTT
    // pools, which means the rows in the cache can be matched by visiting each
    // constraint type in the order they appear in the tuple.
    size_t con_idx = 0;
    size_t row_idx = 0;

    std::apply([&](auto ... c) {
        (update_impulse<decltype(c)>(registry, cache, con_idx, row_idx), ...);
    }, constraints_tuple);
}

solver::solver(entt::registry &registry)
    : m_registry(&registry)
{
    registry.on_construct<linvel>().connect<&entt::registry::emplace<delta_linvel>>();
    registry.on_construct<angvel>().connect<&entt::registry::emplace<delta_angvel>>();
}

template<typename C>
void prepare_constraints_seq(entt::registry &registry, row_cache_sparse &cache, scalar dt, bool mt) {
    auto body_view = registry.view<position, orientation,
                                   linvel, angvel,
                                   mass_inv, inertia_world_inv,
                                   delta_linvel, delta_angvel>();
    auto origin_view = registry.view<origin>();
    auto con_view = registry.view<C>(entt::exclude_t<sleeping_tag, disabled_tag>{});

    auto for_loop_body = [&registry, body_view, con_view, origin_view, &cache, dt](entt::entity entity, size_t index) {
        auto [con] = con_view.get(entity);
        auto [posA, ornA, linvelA, angvelA, inv_mA, inv_IA, dvA, dwA] = body_view.get(con.body[0]);
        auto [posB, ornB, linvelB, angvelB, inv_mB, inv_IB, dvB, dwB] = body_view.get(con.body[1]);

        auto originA = origin_view.contains(con.body[0]) ? origin_view.get<origin>(con.body[0]) : static_cast<vector3>(posA);
        auto originB = origin_view.contains(con.body[1]) ? origin_view.get<origin>(con.body[1]) : static_cast<vector3>(posB);

        auto &entry = cache.entries[index];
        entry.count = 0;

        prepare_constraint(registry, entity, con, entry, dt,
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
void prepare_constraints_async(entt::registry &registry, row_cache_sparse &cache, scalar dt, const job &decr_job) {
    auto body_view = registry.view<position, orientation,
                                   linvel, angvel,
                                   mass_inv, inertia_world_inv,
                                   delta_linvel, delta_angvel>();
    auto origin_view = registry.view<origin>();
    auto con_view = registry.view<C>(entt::exclude_t<sleeping_tag, disabled_tag>{});
    auto &dispatcher = job_dispatcher::global();

    parallel_for_each_async(dispatcher, con_view.begin(), con_view.end(), decr_job,
        [&registry, body_view, con_view, origin_view, &cache, dt](entt::entity entity, size_t index) {
            auto [con] = con_view.get(entity);
            auto [posA, ornA, linvelA, angvelA, inv_mA, inv_IA, dvA, dwA] = body_view.get(con.body[0]);
            auto [posB, ornB, linvelB, angvelB, inv_mB, inv_IB, dvB, dwB] = body_view.get(con.body[1]);

            auto originA = origin_view.contains(con.body[0]) ? origin_view.get<origin>(con.body[0]) : static_cast<vector3>(posA);
            auto originB = origin_view.contains(con.body[1]) ? origin_view.get<origin>(con.body[1]) : static_cast<vector3>(posB);

            auto &entry = cache.entries[index];
            entry.count = 0;

            prepare_constraint(registry, entity, con, entry, dt,
                               originA, posA, ornA, linvelA, angvelA, inv_mA, inv_IA, dvA, dwA,
                               originB, posB, ornB, linvelB, angvelB, inv_mB, inv_IB, dvB, dwB);
        });
}

template<>
void prepare_constraints_seq<null_constraint>(entt::registry &, row_cache_sparse &, scalar, bool) {}

template<>
void prepare_constraints_async<null_constraint>(entt::registry &, row_cache_sparse &, scalar, const job &) {}

bool solver::prepare_constraints(const job &completion_job, scalar dt) {
    // Resize the row cache for each constraint type to fit the rows of
    // each constraint of each type.
    std::apply([&](auto ... c) {
        size_t idx = 0;
        (m_row_cache_sparse[idx++].entries.resize(view_size(m_registry->view<decltype(c)>(entt::exclude_t<sleeping_tag, disabled_tag>{}))), ...);
    }, constraints_tuple);

    // Only constraint types that exist in a sizable number will be
    // prepared in parallel.
    // Calculate how many constraint types will be prepared asynchronously.
    unsigned num_async = 0;
    const size_t max_sequential_size = 2;
    for (auto &cache : m_row_cache_sparse) {
        if (cache.entries.size() > max_sequential_size) {
            ++num_async;
        }
    }

    // Prepare the sequential ones first.
    std::apply([&](auto ... c) {
        size_t idx = 0;
        (((m_row_cache_sparse[idx].entries.size() <= max_sequential_size ?
            prepare_constraints_seq<decltype(c)>(*m_registry, m_row_cache_sparse[idx], dt, false) : void(0)), ++idx), ...);
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
            (((m_row_cache_sparse[idx].entries.size() > max_sequential_size ?
                prepare_constraints_async<decltype(c)>(*m_registry, m_row_cache_sparse[idx], dt, decrement_job) : void(0)), ++idx), ...);
        }, constraints_tuple);

        return false;
    }

    return true;
}

void solver::prepare_constraints_sequential(bool mt, scalar dt) {
    std::apply([&](auto ... c) {
        size_t idx = 0;
        (m_row_cache_sparse[idx++].entries.resize(view_size(m_registry->view<decltype(c)>(entt::exclude_t<sleeping_tag, disabled_tag>{}))), ...);
    }, constraints_tuple);

    std::apply([&](auto ... c) {
        size_t idx = 0;
        ((prepare_constraints_seq<decltype(c)>(*m_registry, m_row_cache_sparse[idx], dt, mt), ++idx), ...);
    }, constraints_tuple);
}

void solver::pack_rows() {
    m_row_cache.clear();

    for (auto &cache : m_row_cache_sparse) {
        for (auto &entry : cache.entries) {
            for (size_t i = 0; i < entry.count; ++i) {
                m_row_cache.rows.push_back(entry.rows[i]);
            }
            m_row_cache.con_num_rows.push_back(entry.count);
        }
    }
}

bool solver::update(const job &completion_job) {
    auto &registry = *m_registry;
    auto &settings = registry.ctx().at<edyn::settings>();
    auto dt = settings.fixed_dt;

    switch (m_state) {
    case state::begin:
        m_row_cache.clear();
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
        m_state = state::pack_rows;
        if (prepare_constraints(completion_job, dt)) {
            return update(completion_job);
        } else {
            return false;
        }
        break;

    case state::pack_rows:
        pack_rows();
        m_state = state::solve_constraints;
        return update(completion_job);
        break;

    case state::solve_constraints:
        for (unsigned i = 0; i < settings.num_solver_velocity_iterations; ++i) {
            // Prepare constraints for iteration.
            //iterate_constraints(registry, m_row_cache, dt);

            // Solve rows.
            for (auto &row : m_row_cache.rows) {
                auto delta_impulse = solve(row);
                apply_impulse(delta_impulse, row);
            }
        }
        m_state = state::apply_delta;
        return update(completion_job);
        break;

    case state::apply_delta:
        // Apply constraint velocity correction.
        for (auto [e, v, w, dv, dw] : registry.view<linvel, angvel, delta_linvel, delta_angvel, dynamic_tag>(entt::exclude_t<sleeping_tag>{}).each()) {
            v += dv;
            w += dw;
            dv = vector3_zero;
            dw = vector3_zero;
        }
        m_state = state::assign_applied_impulse;
        return update(completion_job);
        break;

    case state::assign_applied_impulse:
        update_impulses(registry, m_row_cache);
        m_state = state::integrate_velocity;
        return update(completion_job);
        break;

    case state::integrate_velocity:
        // Integrate velocities to obtain new transforms.
        integrate_linvel(registry, dt);
        integrate_angvel(registry, dt);
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
    auto &settings = registry.ctx().at<edyn::settings>();
    auto dt = settings.fixed_dt;

    m_row_cache.clear();
    solve_restitution(registry, dt);
    apply_gravity(registry, dt);
    prepare_constraints_sequential(mt, dt);
    pack_rows();

    for (unsigned i = 0; i < settings.num_solver_velocity_iterations; ++i) {
        // Prepare constraints for iteration.
        //iterate_constraints(registry, m_row_cache, dt);

        // Solve rows.
        for (auto &row : m_row_cache.rows) {
            auto delta_impulse = solve(row);
            apply_impulse(delta_impulse, row);
        }
    }

    // Apply constraint velocity correction.
    for (auto [e, v, w, dv, dw] : registry.view<linvel, angvel, delta_linvel, delta_angvel, dynamic_tag>(entt::exclude_t<sleeping_tag>{}).each()) {
        v += dv;
        w += dw;
        dv = vector3_zero;
        dw = vector3_zero;
    }

    update_impulses(registry, m_row_cache);

    // Integrate velocities to obtain new transforms.
    integrate_linvel(registry, dt);
    integrate_angvel(registry, dt);

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
