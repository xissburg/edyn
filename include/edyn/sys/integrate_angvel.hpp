#ifndef EDYN_SYS_INTEGRATE_ANGVEL_HPP
#define EDYN_SYS_INTEGRATE_ANGVEL_HPP

#include <entt/entt.hpp>
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/dynamics/island_util.hpp"

namespace edyn {

inline void integrate_angvel(entt::registry &registry, scalar dt) {
    auto view = registry.view<dynamic_tag, orientation, const angvel>(exclude_sleeping);
    view.each([&] (auto, auto, orientation &orn, const angvel &vel) {
        orn = integrate(orn, vel, dt);
    });
}

}

#endif // EDYN_SYS_INTEGRATE_ANGVEL_HPP