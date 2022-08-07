#ifndef EDYN_DYNAMICS_SOLVER_HPP
#define EDYN_DYNAMICS_SOLVER_HPP

#include <memory>
#include <tuple>
#include <vector>
#include <entt/entity/fwd.hpp>
#include "edyn/math/scalar.hpp"
#include "edyn/dynamics/row_cache.hpp"
#include "edyn/constraints/constraint.hpp"
#include "edyn/parallel/atomic_counter.hpp"

namespace edyn {

struct job;

scalar solve(constraint_row &row);

class solver {
    enum class state {
        begin,
        solve_restitution,
        apply_gravity,
        prepare_constraints,
        pack_rows,
        solve_constraints,
        apply_delta,
        assign_applied_impulse,
        integrate_velocity,
        solve_position_constraints,
        finalize,
        done
    };

    bool prepare_constraints(const job &completion_job, scalar dt);
    void pack_rows();

public:
    solver(entt::registry &);

    bool update(const job &completion_job);

private:
    entt::registry *m_registry;
    row_cache m_row_cache;
    std::array<row_cache_sparse, std::tuple_size_v<constraints_tuple_t>> m_row_cache_sparse;
    state m_state;
    std::unique_ptr<atomic_counter> m_counter;
};

}

#endif // EDYN_DYNAMICS_SOLVER_HPP
