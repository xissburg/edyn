#ifndef EDYN_SYS_UPDATE_PRESENT_POSITION_HPP
#define EDYN_SYS_UPDATE_PRESENT_POSITION_HPP

#include <entt/entt.hpp>
#include "edyn/comp/position.hpp"
#include "edyn/comp/present_position.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/dynamics/island_util.hpp"

namespace edyn {

void update_present_position(entt::registry &registry, scalar dt) {
    auto view = registry.view<present_position, const position, const linvel>(exclude_sleeping);
    view.each([dt] (auto, present_position &p_pos, const position &pos, const linvel &vel) {
        p_pos = pos + vel * dt;
    });
}

}

#endif // EDYN_SYS_UPDATE_PRESENT_POSITION_HPP