#ifndef EDYN_SYS_APPLY_GRAVITY_HPP
#define EDYN_SYS_APPLY_GRAVITY_HPP

#include <entt/fwd.hpp>
#include "edyn/math/scalar.hpp"

namespace edyn {

void apply_gravity(entt::registry &, scalar dt);

}

#endif // EDYN_SYS_APPLY_GRAVITY_HPP