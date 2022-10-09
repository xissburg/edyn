#ifndef EDYN_SYS_APPLY_GRAVITY_HPP
#define EDYN_SYS_APPLY_GRAVITY_HPP

#include <entt/entity/registry.hpp>
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/gravity.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/util/island_util.hpp"

namespace edyn {

inline void apply_gravity(entt::registry &registry, scalar dt) {
    auto view = registry.view<linvel, gravity, dynamic_tag>(exclude_sleeping_disabled);
    view.each([&](linvel &vel, gravity &g) {
        vel += g * dt;
    });
}

}

#endif // EDYN_SYS_APPLY_GRAVITY_HPP
