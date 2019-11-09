#ifndef EDYN_SYS_PHYSICS_HPP
#define EDYN_SYS_PHYSICS_HPP

#include <entt/entt.hpp>
#include "integrate_linvel.hpp"
#include "integrate_linacc.hpp"

namespace edyn {

void update(entt::registry& registry, edyn::scalar dt) {
    integrate_linacc(registry, dt);
    integrate_linvel(registry, dt);
}

}

#endif // EDYN_SYS_PHYSICS_HPP