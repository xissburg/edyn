#include "edyn/dynamics/island_solver.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/center_of_mass.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/origin.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/config/config.h"
#include "edyn/config/constants.hpp"
#include "edyn/config/execution_mode.hpp"
#include "edyn/constraints/constraint.hpp"
#include "edyn/constraints/constraint_row_friction.hpp"
#include "edyn/constraints/contact_constraint.hpp"
#include "edyn/dynamics/island_constraint_entities.hpp"
#include "edyn/dynamics/position_solver.hpp"
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
#include "edyn/config/config.h"
#include <entt/entity/fwd.hpp>
#include <entt/entity/registry.hpp>
#include <cstdint>
#include <iterator>
#include <tuple>
#include <type_traits>
#include <utility>

namespace edyn {

static void island_solver_job_func(job::data_type &data);

enum class island_solver_state : uint8_t {
    pack_rows,
    solve_constraints,
    assign_applied_impulses,
    apply_solution,
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

    void decrement_counter() {
        EDYN_ASSERT((counter != nullptr && counter_sync == nullptr) ||
                    (counter == nullptr && counter_sync != nullptr));
        if (counter) {
            counter->decrement();
        } else {
            counter_sync->decrement();
        }
    }
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
        apply_row_impulse(delta_impulse, row);
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

template<typename C>
void insert_rows(entt::registry &registry, row_cache &cache, const entt::sparse_set &entities,
                 island_constraint_entities &constraint_entities) {
    auto prep_view = registry.view<constraint_row_prep_cache>();
    auto con_view = registry.view<C>();
    auto con_idx = tuple_index_of<unsigned, C>(constraints_tuple);

    for (auto entity : entities) {
        if (!con_view.contains(entity)) {
            continue;
        }

        EDYN_ASSERT((!registry.any_of<disabled_tag>(entity)));
        EDYN_ASSERT((!registry.any_of<sleeping_tag>(entity)));

        // Insert entity into array located at the constraint type index.
        constraint_entities.entities[con_idx].push_back(entity);

        auto [prep_cache] = prep_view.get(entity);

        // Insert the number of rows for the current constraint before consuming.
        cache.con_num_rows.push_back(prep_cache.current_num_rows());

        // Insert all constraint rows into island row cache. Since an entity
        // can have multiple constraints (of different types), these could be
        // rows of more than one constraint.
        prep_cache.consume_rows([&](constraint_row_prep_cache::element &elem) {
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
        });
    }
}

void pack_rows(entt::registry &registry, row_cache &cache, const entt::sparse_set &entities,
               island_constraint_entities &constraint_entities) {
    cache.clear();

    for (auto &ents : constraint_entities.entities) {
        ents.clear();
    }

    std::apply([&](auto ... c) {
        (insert_rows<decltype(c)>(registry, cache, entities, constraint_entities), ...);
    }, constraints_tuple);

    warm_start(cache);
}

template<typename C>
void update_impulse(entt::registry &registry, const std::vector<entt::entity> &entities,
                    row_cache &cache, size_t &con_idx, size_t &row_idx, size_t &friction_row_idx,
                    size_t &rolling_row_idx, size_t &spinning_row_idx) {
    auto con_view = registry.view<C>();
    auto manifold_view = registry.view<contact_manifold>();
    std::vector<scalar> applied_impulses;

    for (auto entity : entities) {
        auto [con] = con_view.get(entity);
        auto num_rows = cache.con_num_rows[con_idx];

        if constexpr(std::is_same_v<C, contact_constraint>) {
            auto [manifold] = manifold_view.get(entity);

            for (size_t i = 0; i < num_rows; ++i) {
                auto flags = cache.flags[row_idx];

                con.store_applied_impulse(cache.rows[row_idx++].impulse, i, manifold);

                if (flags & constraint_row_flag_friction) {
                    auto &friction_row = cache.friction[friction_row_idx++];
                    con.store_friction_impulse(friction_row.row[0].impulse, friction_row.row[1].impulse, i, manifold);
                }

                if (flags & constraint_row_flag_rolling_friction) {
                    auto &roll_row = cache.rolling[rolling_row_idx++];
                    con.store_rolling_impulse(roll_row.row[0].impulse, roll_row.row[1].impulse, i, manifold);
                }

                if (flags & constraint_row_flag_spinning_friction) {
                    auto &spin_row = cache.spinning[spinning_row_idx++];
                    con.store_spinning_impulse(spin_row.impulse, i, manifold);
                }
            }
        } else {
            applied_impulses.reserve(num_rows);

            for (size_t i = 0; i < num_rows; ++i) {
                applied_impulses.push_back(cache.rows[row_idx++].impulse);
            }

            con.store_applied_impulses(applied_impulses);
        }

        applied_impulses.clear();
        ++con_idx;
    }
}

template<typename... C, size_t... Ints>
void update_impulses(entt::registry &registry, const island_constraint_entities &constraint_entities,
                     row_cache &cache, size_t &con_idx, size_t &row_idx, size_t &friction_row_idx,
                     size_t &rolling_row_idx, size_t &spinning_row_idx,
                     [[maybe_unused]] std::tuple<C...>,
                     std::index_sequence<Ints...>) {
    // The entities at the i-th array in `constraint_entities` contains the
    // entities that are known to have a constraint of type `C`, which were
    // inserted in the same order they entered the row cache so a direct 1-to-1
    // correspondence can be made.
    (update_impulse<C>(registry, constraint_entities.entities[Ints], cache,
                       con_idx, row_idx, friction_row_idx, rolling_row_idx, spinning_row_idx), ...);
}

void assign_applied_impulses(entt::registry &registry, row_cache &cache,
                             const island_constraint_entities &constraint_entities) {
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

    update_impulses(registry, constraint_entities, cache,
                    con_idx, row_idx, friction_row_idx, rolling_row_idx, spinning_row_idx,
                    constraints_tuple, std::make_index_sequence<std::tuple_size_v<constraints_tuple_t>>());
}

// Check if type implements the solve_position function.
// Reference: https://stackoverflow.com/questions/257288/templated-check-for-the-existence-of-a-class-member-function
template<typename T>
class has_solve_position {
    typedef char yes;
    struct no { char x[2]; };
    template<typename C> static yes test(decltype(&C::solve_position));
    template<typename C> static no test(...);
public:
    static constexpr bool value = sizeof(test<T>(0)) == sizeof(yes);
};

template<typename C, typename BodyView, typename OriginView>
scalar solve_position_constraints_each(entt::registry &registry, const std::vector<entt::entity> &entities,
                                       const BodyView &body_view, const OriginView &origin_view) {
    auto max_error = scalar(0);

    if constexpr(has_solve_position<C>::value) {
        auto con_view = registry.view<C>();
        auto manifold_view = registry.view<contact_manifold>();
        auto solver = position_solver{};

        for (auto entity : entities) {
            auto [con] = con_view.get(entity);
            auto [posA, ornA, inv_mA, inv_IA, inv_IA_local] = body_view.get(con.body[0]);
            auto [posB, ornB, inv_mB, inv_IB, inv_IB_local] = body_view.get(con.body[1]);

            solver.posA = &posA;
            solver.posB = &posB;
            solver.ornA = &ornA;
            solver.ornB = &ornB;
            solver.inv_mA = inv_mA;
            solver.inv_mB = inv_mB;
            solver.inv_IA = &inv_IA;
            solver.inv_IB = &inv_IB;
            solver.inv_IA_local = &inv_IA_local;
            solver.inv_IB_local = &inv_IB_local;

            if (origin_view.contains(con.body[0])) {
                solver.originA = &origin_view.template get<origin>(con.body[0]);
                solver.comA = origin_view.template get<center_of_mass>(con.body[0]);
            } else {
                solver.originA = nullptr;
            }

            if (origin_view.contains(con.body[1])) {
                solver.originB = &origin_view.template get<origin>(con.body[1]);
                solver.comB = origin_view.template get<center_of_mass>(con.body[1]);
            } else {
                solver.originB = nullptr;
            }

            if constexpr(std::is_same_v<std::decay_t<C>, contact_constraint>) {
                auto [manifold] = manifold_view.get(entity);
                con.solve_position(solver, manifold);
            } else {
                con.solve_position(solver);
            }

            max_error = std::max(solver.max_error, max_error);
        }
    }

    return max_error;
}

template<typename... Args>
constexpr auto max_variadic(Args &&...args) {
    return std::max({std::forward<Args>(args)...});
}

template<typename... C, size_t... Ints>
scalar solve_position_constraints_indexed(entt::registry &registry, const island_constraint_entities &constraint_entities,
                                         [[maybe_unused]] std::tuple<C...>, std::index_sequence<Ints...>) {
    auto body_view = registry.view<position, orientation, mass_inv, inertia_world_inv, inertia_inv>();
    auto origin_view = registry.view<origin, center_of_mass>();
    return max_variadic(solve_position_constraints_each<C>(registry, constraint_entities.entities[Ints], body_view, origin_view)...);
}

template<typename... C>
scalar solve_position_constraints(entt::registry &registry, const island_constraint_entities &constraint_entities,
                                  const std::tuple<C...> &constraints) {
    return solve_position_constraints_indexed(registry, constraint_entities, constraints,
                                              std::make_index_sequence<sizeof...(C)>());
}

static bool solve_position_constraints(entt::registry &registry, const island_constraint_entities &constraint_entities) {
    auto error = solve_position_constraints(registry, constraint_entities, constraints_tuple);
    return error < scalar(0.005);
}

bool apply_solution(entt::registry &registry, scalar dt, const entt::sparse_set &entities,
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

    if (entities.size() <= max_sequential_size || mode == execution_mode::sequential) {
        for (auto &entity : entities) {
            for_loop_body(entity);
        }
        return true;
    } else if (mode == execution_mode::sequential_multithreaded) {
        auto &dispatcher = job_dispatcher::global();
        parallel_for_each(dispatcher, entities.begin(), entities.end(), for_loop_body);
        return true;
    } else {
        EDYN_ASSERT(mode == execution_mode::asynchronous);
        auto &dispatcher = job_dispatcher::global();
        parallel_for_each_async(dispatcher, entities.begin(), entities.end(), *completion_job, for_loop_body);
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
        auto &constraint_entities = ctx.registry->get<island_constraint_entities>(ctx.island_entity);
        auto &cache = ctx.registry->get<row_cache>(ctx.island_entity);
        pack_rows(*ctx.registry, cache, island.edges, constraint_entities);

        ctx.state = island_solver_state::solve_constraints;
        ctx.iteration = 0;
        dispatch_solver(ctx);
        break;
    }
    case island_solver_state::solve_constraints: {
        auto &cache = ctx.registry->get<row_cache>(ctx.island_entity);
        solve(cache);

        ++ctx.iteration;

        if (ctx.iteration >= ctx.num_iterations) {
            ctx.state = island_solver_state::apply_solution;
        }

        dispatch_solver(ctx);
        break;
    }
    case island_solver_state::apply_solution: {
        auto &island = ctx.registry->get<edyn::island>(ctx.island_entity);
        ctx.state = island_solver_state::assign_applied_impulses;

        if (apply_solution(*ctx.registry, ctx.dt, island.nodes,
                           execution_mode::asynchronous, make_solver_job(ctx))) {
            dispatch_solver(ctx);
        }
        break;
    }
    case island_solver_state::assign_applied_impulses: {
        auto &cache = ctx.registry->get<row_cache>(ctx.island_entity);
        auto &constraint_entities = ctx.registry->get<island_constraint_entities>(ctx.island_entity);
        assign_applied_impulses(*ctx.registry, cache, constraint_entities);

        if (ctx.num_position_iterations > 0) {
            ctx.iteration = 0;
            ctx.state = island_solver_state::solve_position_constraints;
            dispatch_solver(ctx);
        } else {
            // Done.
            ctx.decrement_counter();
        }
        break;
    }
    case island_solver_state::solve_position_constraints: {
        auto &constraint_entities = ctx.registry->get<island_constraint_entities>(ctx.island_entity);

        if (solve_position_constraints(*ctx.registry, constraint_entities) ||
            ++ctx.iteration >= ctx.num_position_iterations) {
            // Done. Decrement atomic counter.
            ctx.decrement_counter();
        } else {
            dispatch_solver(ctx);
        }
        break;
    }
    }
}

void run_island_solver_seq_mt(entt::registry &registry, entt::entity island_entity,
                             unsigned num_iterations, unsigned num_position_iterations,
                             scalar dt, atomic_counter_sync *counter) {
    auto ctx = island_solver_context(registry, island_entity, num_iterations, num_position_iterations, dt, counter);
    dispatch_solver(ctx);
}

void run_island_solver_seq(entt::registry &registry, entt::entity island_entity,
                           unsigned num_iterations, unsigned num_position_iterations,
                           scalar dt) {
    auto &island = registry.get<edyn::island>(island_entity);
    auto &constraint_entities = registry.get<island_constraint_entities>(island_entity);
    auto &cache = registry.get<row_cache>(island_entity);
    pack_rows(registry, cache, island.edges, constraint_entities);

    for (unsigned i = 0; i < num_iterations; ++i) {
        solve(cache);
    }

    const auto exec_mode = execution_mode::sequential;
    apply_solution(registry, dt, island.nodes, exec_mode);

    assign_applied_impulses(registry, cache, constraint_entities);

    for (unsigned i = 0; i < num_position_iterations; ++i) {
        if (solve_position_constraints(registry, constraint_entities)) {
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
