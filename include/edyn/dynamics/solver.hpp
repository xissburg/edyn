#ifndef EDYN_DYNAMICS_SOLVER_HPP
#define EDYN_DYNAMICS_SOLVER_HPP

#include <entt/entt.hpp>
#include "edyn/comp/constraint_row.hpp"

namespace edyn {

class solver {
public:
    solver(entt::registry &);

    void update(scalar dt);
    void solve(constraint_row &);

private:
    entt::registry *registry;
};

}

#endif // EDYN_DYNAMICS_SOLVER_HPP