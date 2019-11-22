#ifndef EDYN_SYS_INTEGRATE_ANGVEL_HPP
#define EDYN_SYS_INTEGRATE_ANGVEL_HPP

#include <entt/entt.hpp>
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/tag.hpp"

namespace edyn {

void integrate_angvel(entt::registry &registry, scalar dt) {
    auto view = registry.view<orientation, const angvel, const dynamic_tag>();
    view.each([&] (auto, orientation &orn, const angvel &vel, [[maybe_unused]] auto) {
        orn = integrate(orn, vel, dt);
    });
}

}

#endif // EDYN_SYS_INTEGRATE_ANGVEL_HPP