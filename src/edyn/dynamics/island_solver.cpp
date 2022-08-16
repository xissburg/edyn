#include "edyn/dynamics/island_solver.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/origin.hpp"
#include "edyn/config/config.h"
#include "edyn/config/constants.hpp"
#include "edyn/config/execution_mode.hpp"
#include "edyn/constraints/constraint.hpp"
#include "edyn/constraints/constraint_row_friction.hpp"
#include "edyn/constraints/contact_constraint.hpp"
#include "edyn/constraints/null_constraint.hpp"
#include "edyn/constraints/prepare_constraints.hpp"
#include "edyn/dynamics/row_cache.hpp"
#include "edyn/parallel/atomic_counter.hpp"
#include "edyn/parallel/atomic_counter_sync.hpp"
#include "edyn/parallel/job_dispatcher.hpp"
#include "edyn/parallel/parallel_for.hpp"
#include "edyn/parallel/parallel_for_async.hpp"
#include "edyn/util/entt_util.hpp"
#include "edyn/util/island_util.hpp"
#include "edyn/util/constraint_util.hpp"
#include "edyn/serialization/entt_s11n.hpp"
#include "edyn/util/tuple_util.hpp"
#include <cstdint>
#include <entt/entity/registry.hpp>
#include <iterator>
#include <tuple>
#include <type_traits>

namespace edyn {

static void island_solver_job_func(job::data_type &data);

enum class island_solver_state : uint8_t {
    pack_rows,
    solve_constraints,
    assign_applied_impulses,
    apply_solution,
    prepare_position_constraints,
    pack_position_rows,
    solve_position_constraints
};

struct island_solver_context {
    entt::registry *registry;
    entt::entity island_entity;
    atomic_counter *counter {nullptr};
    atomic_counter_sync *counter_sync {nullptr};
    scalar dt;
    uint8_t num_iterations;
    uint8_t num_position_iterations;
    uint8_t iteration {};
    island_solver_state state {island_solver_state::pack_rows};

    island_solver_context() = default;

    island_solver_context(entt::registry &registry, entt::entity island_entity,
                          uint8_t num_iterations, uint8_t num_position_iterations,
                          scalar dt, atomic_counter *counter)
        : registry(&registry)
        , island_entity(island_entity)
        , num_iterations(num_iterations)
        , num_position_iterations(num_position_iterations)
        , dt(dt)
        , counter(counter)
    {}

    island_solver_context(entt::registry &registry, entt::entity island_entity,
                          uint8_t num_iterations, uint8_t num_position_iterations,
                          scalar dt, atomic_counter_sync *counter)
        : registry(&registry)
        , island_entity(island_entity)
        , num_iterations(num_iterations)
        , num_position_iterations(num_position_iterations)
        , dt(dt)
        , counter_sync(counter)
    {}
};

template<typename Archive>
void serialize(Archive &archive, island_solver_context &ctx) {
    serialize_pointer(archive, &ctx.registry);
    archive(ctx.island_entity);
    serialize_pointer(archive, &ctx.counter);
    serialize_pointer(archive, &ctx.counter_sync);
    archive(ctx.dt);
    archive(ctx.num_iterations);
    archive(ctx.num_position_iterations);
    archive(ctx.iteration);
    serialize_enum(archive, ctx.state);
}

static void warm_start(row_cache &cache) {
    for (auto &row : cache.rows) {
        warm_start(row);
    }

    for (auto &row : cache.friction) {
        warm_start(row, cache.rows);
    }

    for (auto &row : cache.rolling) {
        warm_start(row, cache.rows);
    }

    for (auto &row : cache.spinning) {
        warm_start(row, cache.rows);
    }
}

static void solve(row_cache &cache) {
    for (auto &row : cache.rows) {
        auto delta_impulse = solve(row);
        apply_impulse(delta_impulse, row);
    }

    for (auto &row : cache.friction) {
        solve_friction(row, cache.rows);
    }

    for (auto &row : cache.rolling) {
        solve_friction(row, cache.rows);
    }

    for (auto &row : cache.spinning) {
        solve_spin_friction(row, cache.rows);
    }
}

static void solve_position(constraint_row_positional &row) {
    auto eff_mass = get_effective_mass(row.J, row.inv_mA, *row.inv_IA, row.inv_mB, *row.inv_IB);
    auto correction = row.error * contact_position_correction_rate * eff_mass;

    *row.posA += row.inv_mA * row.J[0] * correction;
    *row.posB += row.inv_mB * row.J[2] * correction;

    // Use quaternion derivative to apply angular correction which should
    // be good enough for small angles.
    auto angular_correctionA = *row.inv_IA * row.J[1] * correction;
    *row.ornA += quaternion_derivative(*row.ornA, angular_correctionA);
    *row.ornA = normalize(*row.ornA);

    auto angular_correctionB = *row.inv_IB * row.J[3] * correction;
    *row.ornB += quaternion_derivative(*row.ornB, angular_correctionB);
    *row.ornB = normalize(*row.ornB);

    auto basisA = to_matrix3x3(*row.ornA);
    *row.inv_IA = basisA * *row.inv_IA * transpose(basisA);

    auto basisB = to_matrix3x3(*row.ornB);
    *row.inv_IB = basisB * *row.inv_IB * transpose(basisB);
}

static bool solve(row_cache_positional &cache) {
    auto error = scalar(0);

    for (auto &row : cache.rows) {
        solve_position(row);
        error = std::max(std::abs(row.error), error);
    }

    return error < scalar(0.005);
}

template<typename It>
void insert_rows(entt::registry &registry, row_cache &cache, It first, It last) {
    auto prep_view = registry.view<constraint_row_prep_cache>();

    for (; first != last; ++first) {
        auto entity = *first;

        if (!prep_view.contains(entity)) {
            continue;
        }

        auto [prep_cache] = prep_view.get(entity);

        // Insert all constraint rows into island row cache. Since an entity
        // can have multiple constraints (of different types), these could be
        // rows of more than one constraint.
        for (unsigned i = 0; i < prep_cache.num_rows; ++i) {
            auto &elem = prep_cache.rows[i];
            auto normal_row_index = cache.rows.size();
            cache.rows.push_back(elem.row);
            cache.flags.push_back(elem.flags);

            if (elem.flags & constraint_row_flag_friction) {
                cache.friction.push_back(elem.friction);
                cache.friction.back().normal_row_index = normal_row_index;
            }

            if (elem.flags & constraint_row_flag_rolling_friction) {
                cache.rolling.push_back(elem.rolling);
                cache.rolling.back().normal_row_index = normal_row_index;
            }

            if (elem.flags & constraint_row_flag_spinning_friction) {
                cache.spinning.push_back(elem.spinning);
                cache.spinning.back().normal_row_index = normal_row_index;
            }
        }

        // Then insert the number of rows for each constraint.
        for (unsigned i = 0; i < prep_cache.num_constraints; ++i) {
            cache.con_num_rows.push_back(prep_cache.rows_per_constraint[i]);
        }

        prep_cache.clear();
    }
}

template<typename It>
void pack_rows(entt::registry &registry, row_cache &cache, It first, It last) {
    cache.clear();
    insert_rows(registry, cache, first, last);
    warm_start(cache);
}

template<typename C>
void update_impulse(entt::registry &registry, entt::entity entity,
                    entt::basic_view<entt::entity, entt::get_t<C>, entt::exclude_t<>> &con_view,
                    entt::basic_view<entt::entity, entt::get_t<contact_manifold>, entt::exclude_t<>> &manifold_view,
                    row_cache &cache, size_t &con_idx, size_t &row_idx, size_t &friction_row_idx,
                    size_t &rolling_row_idx, size_t &spinning_row_idx) {
    if (!con_view.contains(entity)) {
        return;
    }

    auto [con] = con_view.get(entity);
    auto num_rows = cache.con_num_rows[con_idx];

    constexpr auto friction_start_index = max_contacts;
    constexpr auto rolling_start_index = max_contacts + max_contacts * 2;
    constexpr auto spinning_start_index = rolling_start_index + max_contacts * 2;

    // Check if the constraint `impulse` member is a scalar, otherwise
    // an array is expected.
    if constexpr(std::is_same_v<decltype(con.impulse), scalar>) {
        EDYN_ASSERT(num_rows == 1);
        con.impulse = cache.rows[row_idx].impulse;
    } else {
        EDYN_ASSERT(con.impulse.size() >= num_rows);
        for (size_t i = 0; i < num_rows; ++i) {
            con.impulse[i] = cache.rows[row_idx + i].impulse;

            auto flags = cache.flags[row_idx + i];

            if (flags & constraint_row_flag_friction) {
                auto &friction_row = cache.friction[friction_row_idx++];
                for (unsigned j = 0; j < 2; ++j) {
                    con.impulse[friction_start_index + i * 2 + j] = friction_row.row[j].impulse;
                }
            }

            if (flags & constraint_row_flag_rolling_friction) {
                auto &roll_row = cache.rolling[rolling_row_idx++];
                for (unsigned j = 0; j < 2; ++j) {
                    con.impulse[rolling_start_index + i * 2 + j] = roll_row.row[j].impulse;
                }
            }

            if (flags & constraint_row_flag_spinning_friction) {
                auto &spin_row = cache.spinning[spinning_row_idx++];
                con.impulse[spinning_start_index + i] = spin_row.impulse;
            }
        }
    }

    if constexpr(std::is_same_v<C, contact_constraint>) {
        auto [manifold] = manifold_view.get(entity);
        EDYN_ASSERT(manifold.num_points == num_rows);
        for (size_t i = 0; i < num_rows; ++i) {
            auto &pt = manifold.get_point(i);
            pt.normal_impulse = con.impulse[i];

            for (int j = 0; j < 2; ++j) {
                pt.friction_impulse[j] = con.impulse[friction_start_index + i * 2 + j];
            }

            for (int j = 0; j < 2; ++j) {
                pt.rolling_friction_impulse[j] = con.impulse[rolling_start_index + i * 2 + j];
            }

            pt.spin_friction_impulse = con.impulse[spinning_start_index + i];
        }
    }

    row_idx += num_rows;
    ++con_idx;
}

// No-op for null constraints.
template<>
void update_impulse<null_constraint>(entt::registry &, entt::entity,
                    entt::basic_view<entt::entity, entt::get_t<null_constraint>, entt::exclude_t<>> &,
                    entt::basic_view<entt::entity, entt::get_t<contact_manifold>, entt::exclude_t<>> &,
                    row_cache &, size_t &, size_t &, size_t &, size_t &, size_t &) {}

template<typename It>
void assign_applied_impulses(entt::registry &registry, row_cache &cache, It first, It last) {
    // Assign impulses from constraint rows back into the constraints. The rows
    // are inserted into the cache for each constraint type in the order they're
    // found in `constraints_tuple` and in the same order they're in their EnTT
    // pools, which means the rows in the cache can be matched by visiting each
    // constraint type in the order they appear in the tuple.
    size_t con_idx = 0;
    size_t row_idx = 0;
    size_t friction_row_idx = 0;
    size_t rolling_row_idx = 0;
    size_t spinning_row_idx = 0;
    auto con_view_tuple = get_tuple_of_views(registry, constraints_tuple);
    auto manifold_view = registry.view<contact_manifold>();

    for (; first != last; ++first) {
        auto entity = *first;
        std::apply([&](auto &&... con_view) {
            (update_impulse(registry, entity, con_view, manifold_view, cache,
                            con_idx, row_idx, friction_row_idx, rolling_row_idx, spinning_row_idx), ...);
        }, con_view_tuple);
    }
}

template<typename C, typename BodyView, typename OriginView>
void invoke_prepare_position_constraint(entt::registry &registry, entt::entity entity,
                                        constraint_row_positional_prep_cache &cache,
                                        const entt::basic_view<entt::entity, entt::get_t<C>, entt::exclude_t<>> &con_view,
                                        const BodyView &body_view, const OriginView &origin_view) {
    if constexpr(std::is_same_v<C, null_constraint>) {
        return;
    }

    if (!con_view.contains(entity)) {
        return;
    }

    auto [con] = con_view.get(entity);
    auto [posA, ornA, inv_mA, inv_IA] = body_view.get(con.body[0]);
    auto [posB, ornB, inv_mB, inv_IB] = body_view.get(con.body[1]);

    auto originA = origin_view.contains(con.body[0]) ?
        origin_view.template get<origin>(con.body[0]) : static_cast<vector3>(posA);
    auto originB = origin_view.contains(con.body[1]) ?
        origin_view.template get<origin>(con.body[1]) : static_cast<vector3>(posB);

    cache.add_constraint();

    prepare_position_constraint(registry, entity, con, cache,
                                originA, posA, ornA, inv_mA, inv_IA,
                                originB, posB, ornB, inv_mB, inv_IB);
}

template<typename It>
static void prepare_position_constraints(entt::registry &registry, It first, It last) {
    auto body_view = registry.view<position, orientation,
                                   mass_inv, inertia_world_inv>();
    auto origin_view = registry.view<origin>();
    auto cache_view = registry.view<constraint_row_positional_prep_cache>(exclude_sleeping_disabled);
    auto con_view_tuple = get_tuple_of_views(registry, constraints_tuple);

    auto for_loop_body = [&registry, body_view, cache_view, origin_view, con_view_tuple](entt::entity entity) {
        auto &cache = cache_view.get<constraint_row_positional_prep_cache>(entity);

        std::apply([&](auto &&... con_view) {
            (invoke_prepare_position_constraint(registry, entity, cache, con_view, body_view, origin_view), ...);
        }, con_view_tuple);
    };

    for (; first != last; ++first) {
        auto entity = *first;

        if (cache_view.contains(entity)) {
            for_loop_body(entity);
        }
    }
}

template<typename It>
void pack_rows_positional(entt::registry &registry, row_cache_positional &cache, It first, It last) {
    cache.rows.clear();
    auto cache_view = registry.view<constraint_row_positional_prep_cache>();

    for (; first != last; ++first) {
        auto entity = *first;

        if (cache_view.contains(entity)) {
            auto [prep_cache] = cache_view.get(entity);
            auto first = prep_cache.rows.begin();
            auto last = first;
            std::advance(last, prep_cache.num_rows);
            cache.rows.insert(cache.rows.end(), first, last);
            prep_cache.clear();
        }
    }
}

template<typename It>
bool apply_solution(entt::registry &registry, scalar dt, It first, It last,
                    execution_mode mode, std::optional<job> completion_job = {}) {
    auto view = registry.view<position, orientation,
                              linvel, angvel, delta_linvel, delta_angvel,
                              dynamic_tag>();

    auto for_loop_body = [view, dt](entt::entity entity) {
        if (view.contains(entity)) {
            auto [pos, orn, v, w, dv, dw] = view.get(entity);

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
    };

    constexpr auto max_sequential_size = 64;
    auto size = std::distance(view.begin(), view.end());

    if (size <= max_sequential_size || mode == execution_mode::sequential) {
        for (; first != last; ++first) {
            for_loop_body(*first);
        }
        return true;
    } else if (mode == execution_mode::sequential_multithreaded) {
        auto &dispatcher = job_dispatcher::global();
        parallel_for_each(dispatcher, first, last, for_loop_body);
        return true;
    } else {
        EDYN_ASSERT(mode == execution_mode::asynchronous);
        auto &dispatcher = job_dispatcher::global();
        parallel_for_each_async(dispatcher, first, last, *completion_job, for_loop_body);
        return false;
    }
}

static job make_solver_job(island_solver_context &ctx) {
    auto j = job();
    j.func = &island_solver_job_func;
    auto archive = fixed_memory_output_archive(j.data.data(), j.data.size());
    archive(ctx);
    EDYN_ASSERT(!archive.failed());
    return j;
}

static void dispatch_solver(island_solver_context &ctx) {
    auto j = make_solver_job(ctx);
    job_dispatcher::global().async(j);
}

static void island_solver_update(island_solver_context &ctx) {
    switch (ctx.state) {
    case island_solver_state::pack_rows: {
        auto &island = ctx.registry->get<edyn::island>(ctx.island_entity);
        auto &cache = ctx.registry->get<row_cache>(ctx.island_entity);
        pack_rows(*ctx.registry, cache, island.edges.begin(), island.edges.end());
        ctx.state = island_solver_state::solve_constraints;
        ctx.iteration = 0;
        dispatch_solver(ctx);
    }
    case island_solver_state::solve_constraints: {
        auto &cache = ctx.registry->get<row_cache>(ctx.island_entity);
        solve(cache);

        ++ctx.iteration;

        if (ctx.iteration >= ctx.num_iterations) {
            ctx.state = island_solver_state::apply_solution;
        }

        dispatch_solver(ctx);
    }
    case island_solver_state::apply_solution: {
        auto &island = ctx.registry->get<edyn::island>(ctx.island_entity);
        ctx.state = island_solver_state::assign_applied_impulses;

        if (apply_solution(*ctx.registry, ctx.dt, island.nodes.begin(), island.nodes.end(),
                           execution_mode::asynchronous, make_solver_job(ctx))) {
            dispatch_solver(ctx);
        }
    }
    case island_solver_state::assign_applied_impulses: {
        auto &island = ctx.registry->get<edyn::island>(ctx.island_entity);
        auto &cache = ctx.registry->get<row_cache>(ctx.island_entity);
        assign_applied_impulses(*ctx.registry, cache, island.edges.begin(), island.edges.end());

        if (ctx.num_position_iterations > 0) {
            ctx.iteration = 0;
            ctx.state = island_solver_state::prepare_position_constraints;
        }

        dispatch_solver(ctx);
    }
    case island_solver_state::prepare_position_constraints: {
        auto &island = ctx.registry->get<edyn::island>(ctx.island_entity);
        prepare_position_constraints(*ctx.registry, island.edges.begin(), island.edges.end());
        ctx.state = island_solver_state::pack_position_rows;
        dispatch_solver(ctx);
    }
    case island_solver_state::pack_position_rows: {
        auto &island = ctx.registry->get<edyn::island>(ctx.island_entity);
        auto &cache = ctx.registry->get<row_cache_positional>(ctx.island_entity);
        pack_rows_positional(*ctx.registry, cache, island.edges.begin(), island.edges.end());
        ctx.state = island_solver_state::solve_position_constraints;
        dispatch_solver(ctx);
    }
    case island_solver_state::solve_position_constraints: {
        auto &cache = ctx.registry->get<row_cache_positional>(ctx.island_entity);

        if (ctx.iteration++ >= ctx.num_position_iterations || solve(cache)) {
            // Done. Decrement atomic counter.
            EDYN_ASSERT((ctx.counter != nullptr && ctx.counter_sync == nullptr) ||
                        (ctx.counter == nullptr && ctx.counter_sync != nullptr));
            if (ctx.counter) {
                ctx.counter->decrement();
            } else {
                ctx.counter_sync->decrement();
            }
        } else {
            ctx.state = island_solver_state::prepare_position_constraints;
            dispatch_solver(ctx);
        }
    }
    }
}

template<typename AtomicCounterType>
void run_island_solver(entt::registry &registry, entt::entity island_entity,
                       unsigned num_iterations, unsigned num_position_iterations,
                       scalar dt, AtomicCounterType *counter) {
    EDYN_ASSERT(counter != nullptr);
    auto ctx = island_solver_context(registry, island_entity, num_iterations, num_position_iterations, dt, counter);
    dispatch_solver(ctx);
}

void run_island_solver_async(entt::registry &registry, entt::entity island_entity,
                             unsigned num_iterations, unsigned num_position_iterations,
                             scalar dt, atomic_counter *counter) {
    run_island_solver(registry, island_entity, num_iterations, num_position_iterations, dt, counter);
}

void run_island_solver_seq_mt(entt::registry &registry, entt::entity island_entity,
                             unsigned num_iterations, unsigned num_position_iterations,
                             scalar dt, atomic_counter_sync *counter) {
    run_island_solver(registry, island_entity, num_iterations, num_position_iterations, dt, counter);
}

void run_island_solver_seq(entt::registry &registry, entt::entity island_entity,
                           unsigned num_iterations, unsigned num_position_iterations,
                           scalar dt) {
    auto &island = registry.get<edyn::island>(island_entity);
    auto &cache = registry.get<row_cache>(island_entity);
    pack_rows(registry, cache, island.edges.begin(), island.edges.end());

    for (unsigned i = 0; i < num_iterations; ++i) {
        solve(cache);
    }

    const auto exec_mode = execution_mode::sequential;
    apply_solution(registry, dt, island.nodes.begin(), island.nodes.end(), exec_mode);
    assign_applied_impulses(registry, cache, island.edges.begin(), island.edges.end());

    auto &cache_pos = registry.get<row_cache_positional>(island_entity);

    for (unsigned i = 0; i < num_position_iterations; ++i) {
        prepare_position_constraints(registry, island.edges.begin(), island.edges.end());
        pack_rows_positional(registry, cache_pos, island.edges.begin(), island.edges.end());

        if (solve(cache_pos)) {
            break;
        }
    }
}

static void island_solver_job_func(job::data_type &data) {
    auto archive = memory_input_archive(data.data(), data.size());
    island_solver_context ctx;
    archive(ctx);
    island_solver_update(ctx);
}

}
