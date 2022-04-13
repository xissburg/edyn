#ifndef EDYN_SYS_INTEGRATE_LINVEL_HPP
#define EDYN_SYS_INTEGRATE_LINVEL_HPP

#include <entt/entity/registry.hpp>
#include "edyn/comp/position.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/tag.hpp"

namespace edyn {

/**
 * @brief Integrate linear velocity thus updating position for dynamic entities.
 *
 * @param registry The registry to be updated.
 * @param dt The amount of time that has passed since the last invocation.
 */
inline void integrate_linvel(entt::registry &registry, scalar dt) {
    auto view = registry.view<position, linvel, dynamic_tag>();
    view.each([&](position &pos, linvel &vel) {
        pos += vel * dt;
    });
}

}

#endif // EDYN_SYS_INTEGRATE_LINVEL_HPP
