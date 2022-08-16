#ifndef EDYN_DYNAMICS_SOLVER_HPP
#define EDYN_DYNAMICS_SOLVER_HPP

#include <entt/signal/sigh.hpp>
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

class solver final {
    enum class state {
        begin,
        solve_restitution,
        apply_gravity,
        prepare_constraints,
        solve_islands,
        finalize,
        done
    };

public:
    solver(entt::registry &);

    bool update(const job &completion_job);
    void update_sequential(bool mt);

private:
    entt::registry *m_registry;
    std::vector<entt::scoped_connection> m_connections;
    state m_state {state::begin};
    std::unique_ptr<atomic_counter> m_counter;
};

}

#endif // EDYN_DYNAMICS_SOLVER_HPP
