#ifndef EDYN_SYS_INTEGRATE_LINVEL_HPP
#define EDYN_SYS_INTEGRATE_LINVEL_HPP

#include <entt/entt.hpp>
#include "edyn/comp/position.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/dynamics/island_util.hpp"

namespace edyn {

/**
 * @brief Integrate linear velocity thus updating position for dynamic entities.
 * 
 * @param registry The registry to be updated.
 * @param dt The amount of time that has passed since the last invocation.
 */
void integrate_linvel(entt::registry &registry, scalar dt) {
    auto view = registry.view<dynamic_tag, position, const linvel>(exclude_sleeping);
    view.each([&] (auto, auto, position &pos, const linvel &vel) {
        pos += vel * dt;
    });
}

}

#endif // EDYN_SYS_INTEGRATE_LINVEL_HPP