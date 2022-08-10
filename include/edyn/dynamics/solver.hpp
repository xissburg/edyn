#ifndef EDYN_DYNAMICS_SOLVER_HPP
#define EDYN_DYNAMICS_SOLVER_HPP

#include <memory>
#include <tuple>
#include <vector>
#include <entt/entity/fwd.hpp>
#include "edyn/math/scalar.hpp"
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
        solve_constraints,
        apply_solution,
        solve_position_constraints,
        finalize,
        done
    };

    bool prepare_constraints(const job &completion_job, scalar dt);
    void prepare_constraints_sequential(bool mt, scalar dt);
    void pack_rows();

public:
    solver(entt::registry &);

    bool update(const job &completion_job);
    void update_sequential(bool mt);

private:
    entt::registry *m_registry;
    state m_state {state::begin};
    std::unique_ptr<atomic_counter> m_counter;
};

}

#endif // EDYN_DYNAMICS_SOLVER_HPP
