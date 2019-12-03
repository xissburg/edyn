#ifndef EDYN_SYS_UPDATE_PRESENT_ORIENTATION_HPP
#define EDYN_SYS_UPDATE_PRESENT_ORIENTATION_HPP

#include <entt/entt.hpp>
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/present_orientation.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/dynamics/island_util.hpp"

namespace edyn {

void update_present_orientation(entt::registry &registry, scalar dt) {
    auto view = registry.view<present_orientation, const orientation, const angvel>(exclude_sleeping);
    view.each([dt] (auto, present_orientation &p_orn, const orientation &orn, const angvel &vel) {
        p_orn = integrate(orn, vel, dt);
    });
}

}

#endif // EDYN_SYS_UPDATE_PRESENT_ORIENTATION_HPP