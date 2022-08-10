#include "edyn/dynamics/island_solver.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/config/config.h"
#include "edyn/constraints/constraint.hpp"
#include "edyn/constraints/contact_constraint.hpp"
#include "edyn/constraints/null_constraint.hpp"
#include "edyn/dynamics/row_cache.hpp"
#include "edyn/parallel/atomic_counter.hpp"
#include "edyn/parallel/atomic_counter_sync.hpp"
#include "edyn/util/entt_util.hpp"
#include "edyn/util/island_util.hpp"
#include "edyn/util/constraint_util.hpp"
#include "edyn/serialization/entt_s11n.hpp"
#include "edyn/util/tuple_util.hpp"
#include <entt/entity/registry.hpp>
#include <tuple>
#include <type_traits>

namespace edyn {

static void island_solver_job_func(job::data_type &data);

enum class island_solver_state : uint8_t {
    pack_rows,
    solve_constraints,
    assign_applied_impulses
};

struct island_solver_context {
    entt::registry *registry;
    entt::entity island_entity;
    atomic_counter *counter {nullptr};
    atomic_counter_sync *counter_sync {nullptr};
    unsigned num_iterations;
    unsigned iteration {};
    island_solver_state state {island_solver_state::pack_rows};

    island_solver_context() = default;

    island_solver_context(entt::registry &registry, entt::entity island_entity,
                          unsigned num_iterations, atomic_counter *counter)
        : registry(&registry)
        , island_entity(island_entity)
        , num_iterations(num_iterations)
        , counter(counter)
    {}

    island_solver_context(entt::registry &registry, entt::entity island_entity,
                          unsigned num_iterations, atomic_counter_sync *counter)
        : registry(&registry)
        , island_entity(island_entity)
        , num_iterations(num_iterations)
        , counter_sync(counter)
    {}

    island_solver_context(entt::registry &registry, entt::entity island_entity,
                          unsigned num_iterations)
        : registry(&registry)
        , island_entity(island_entity)
        , num_iterations(num_iterations)
    {}
};

template<typename Archive>
void serialize(Archive &archive, island_solver_context &ctx) {
    serialize_pointer(archive, &ctx.registry);
    archive(ctx.island_entity);
    serialize_pointer(archive, &ctx.counter);
    serialize_pointer(archive, &ctx.counter_sync);
    archive(ctx.num_iterations);
    archive(ctx.iteration);
    serialize_enum(archive, ctx.state);
}

template<typename It>
void insert_rows(entt::registry &registry, row_cache &cache, It first, It last) {
    auto prep_view = registry.view<constraint_row_prep_cache>();

    for (; first != last; ++first) {
        auto entity = *first;
        auto [prep_cache] = prep_view.get(entity);

        // Insert all constraint rows into island row cache. Since an entity
        // can have multiple constraints (of different types), these could be
        // rows of more than one constraint.
        if (prep_cache.num_rows > 0) {
            auto end = prep_cache.rows.begin();
            std::advance(end, prep_cache.num_rows);
            cache.rows.insert(cache.rows.end(), prep_cache.rows.begin(), end);
        }

        // Then insert the number of rows for each constraint.
        for (unsigned i = 0; i < prep_cache.num_constraints; ++i) {
            auto num_rows = prep_cache.rows_per_constraint[i];
            cache.con_num_rows.push_back(num_rows);
        }

        prep_cache.clear();
    }
}

template<typename C>
void update_impulse(entt::registry &registry, entt::entity entity,
                    entt::basic_view<entt::entity, entt::get_t<C>, entt::exclude_t<>> &con_view,
                    entt::basic_view<entt::entity, entt::get_t<contact_manifold>, entt::exclude_t<>> &manifold_view,
                    row_cache &cache, size_t &con_idx, size_t &row_idx) {
    if (!con_view.contains(entity)) {
        return;
    }

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

    if constexpr(std::is_same_v<C, contact_constraint>) {
        auto [manifold] = manifold_view.get(entity);
        EDYN_ASSERT(manifold.num_points == num_rows);
        for (size_t i = 0; i < num_rows; ++i) {
            manifold.get_point(i).normal_impulse = con.impulse[i];
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
                    row_cache &, size_t &, size_t &) {}

// Specialization to assign the impulses of friction constraints which are not
// stored in traditional constraint rows.
/* template<>
void update_impulse<contact_constraint>(entt::registry &registry, row_cache &cache, size_t &con_idx, size_t &row_idx) {
    auto con_view = registry.view<contact_constraint, contact_manifold>(exclude_sleeping_disabled);
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

template<typename It>
void update_impulses(entt::registry &registry, row_cache &cache, It first, It last) {
    // Assign impulses from constraint rows back into the constraints. The rows
    // are inserted into the cache for each constraint type in the order they're
    // found in `constraints_tuple` and in the same order they're in their EnTT
    // pools, which means the rows in the cache can be matched by visiting each
    // constraint type in the order they appear in the tuple.
    size_t con_idx = 0;
    size_t row_idx = 0;
    auto con_view_tuple = get_tuple_of_views(registry, constraints_tuple);
    auto manifold_view = registry.view<contact_manifold>();

    for (; first != last; ++first) {
        auto entity = *first;
        std::apply([&](auto &&... con_view) {
            (update_impulse(registry, entity, con_view, manifold_view, cache, con_idx, row_idx), ...);
        }, con_view_tuple);
    }
}

static bool run_island_solver_state_machine(island_solver_context &ctx) {
    switch (ctx.state) {
    case island_solver_state::pack_rows: {
        auto &island = ctx.registry->get<edyn::island>(ctx.island_entity);
        auto &cache = ctx.registry->get<row_cache>(ctx.island_entity);
        cache.clear();

        insert_rows(*ctx.registry, cache, island.edges.begin(), island.edges.end());

        ctx.state = island_solver_state::solve_constraints;
        ctx.iteration = 0;

        return false;
    }
    case island_solver_state::solve_constraints: {
        auto &cache = ctx.registry->get<row_cache>(ctx.island_entity);

        for (auto &row : cache.rows) {
            auto delta_impulse = solve(row);
            apply_impulse(delta_impulse, row);
        }

        ++ctx.iteration;

        if (ctx.iteration >= ctx.num_iterations) {
            ctx.state = island_solver_state::assign_applied_impulses;
        }

        return false;
    }
    case island_solver_state::assign_applied_impulses: {
        auto &island = ctx.registry->get<edyn::island>(ctx.island_entity);
        auto &cache = ctx.registry->get<row_cache>(ctx.island_entity);
        update_impulses(*ctx.registry, cache, island.edges.begin(), island.edges.end());

        return true;
    }
    }
}

static void island_solver_update(island_solver_context &ctx) {
    if (!run_island_solver_state_machine(ctx)) {
        auto j = job();
        j.func = &island_solver_job_func;
        auto archive = fixed_memory_output_archive(j.data.data(), j.data.size());
        archive(ctx);
        job_dispatcher::global().async(j);
    } else {
        EDYN_ASSERT((ctx.counter != nullptr && ctx.counter_sync == nullptr) ||
                    (ctx.counter == nullptr && ctx.counter_sync != nullptr));
        if (ctx.counter) {
            ctx.counter->decrement();
        } else {
            ctx.counter_sync->decrement();
        }
    }
}

void run_island_solver_async(entt::registry &registry, entt::entity island_entity,
                             unsigned int num_iterations, atomic_counter *counter) {
    EDYN_ASSERT(counter != nullptr);
    auto ctx = island_solver_context(registry, island_entity, num_iterations, counter);
    auto j = job();
    j.func = &island_solver_job_func;
    auto archive = fixed_memory_output_archive(j.data.data(), j.data.size());
    archive(ctx);
    job_dispatcher::global().async(j);
}

void run_island_solver_seq_mt(entt::registry &registry, entt::entity island_entity,
                              unsigned int num_iterations, atomic_counter_sync *counter) {
    EDYN_ASSERT(counter != nullptr);
    auto ctx = island_solver_context(registry, island_entity, num_iterations, counter);
    auto j = job();
    j.func = &island_solver_job_func;
    auto archive = fixed_memory_output_archive(j.data.data(), j.data.size());
    archive(ctx);
    job_dispatcher::global().async(j);
}

void run_island_solver_seq(entt::registry &registry, entt::entity island_entity,
                           unsigned int num_iterations) {
    auto ctx = island_solver_context(registry, island_entity, num_iterations);
    while (!run_island_solver_state_machine(ctx));
}

static void island_solver_job_func(job::data_type &data) {
    auto archive = memory_input_archive(data.data(), data.size());
    island_solver_context ctx;
    archive(ctx);
    island_solver_update(ctx);
}

}
