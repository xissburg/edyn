#ifndef EDYN_DYNAMICS_RESTITUTION_SOLVER_HPP
#define EDYN_DYNAMICS_RESTITUTION_SOLVER_HPP

#include "edyn/math/scalar.hpp"
#include <entt/entity/fwd.hpp>

namespace edyn {

void solve_restitution(entt::registry &registry, scalar dt);

}

#endif // EDYN_DYNAMICS_RESTITUTION_SOLVER_HPP
