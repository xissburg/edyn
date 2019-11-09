#ifndef EDYN_SYS_INTEGRATE_LINVEL_HPP
#define EDYN_SYS_INTEGRATE_LINVEL_HPP

#include <entt/entt.hpp>
#include "edyn/comp/position.hpp"
#include "edyn/comp/linvel.hpp"

namespace edyn {

void integrate_linvel(entt::registry& registry, edyn::scalar dt) {
    auto view = registry.view<position, const linvel>();
    view.each([&] (auto, position& pos, const linvel& vel) {
        pos += vel * dt;
    });
}

}


#endif // EDYN_SYS_INTEGRATE_LINVEL_HPP