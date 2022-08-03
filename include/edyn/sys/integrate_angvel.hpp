#ifndef EDYN_SYS_INTEGRATE_ANGVEL_HPP
#define EDYN_SYS_INTEGRATE_ANGVEL_HPP

#include <entt/entity/registry.hpp>
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/tag.hpp"

namespace edyn {

inline void integrate_angvel(entt::registry &registry, scalar dt) {
    auto view = registry.view<orientation, angvel, dynamic_tag>(entt::exclude_t<sleeping_tag, disabled_tag>{});
    view.each([&](orientation &orn, angvel &vel) {
        orn = integrate(orn, vel, dt);
    });
}

}

#endif // EDYN_SYS_INTEGRATE_ANGVEL_HPP
