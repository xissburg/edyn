#ifndef EDYN_DYNAMICS_SOLVER_HPP
#define EDYN_DYNAMICS_SOLVER_HPP

#include <vector>
#include <entt/entity/fwd.hpp>
#include "edyn/math/scalar.hpp"
#include "edyn/dynamics/row_cache.hpp"

namespace edyn {

scalar solve(constraint_row &row);

class solver {
public:
    solver(entt::registry &);
    ~solver();

    void update(scalar dt);

    unsigned velocity_iterations {8};
    unsigned position_iterations {3};

private:
    entt::registry *m_registry;
    row_cache m_row_cache;
};

}

#endif // EDYN_DYNAMICS_SOLVER_HPP
