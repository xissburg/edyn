#ifndef EDYN_SYS_UPDATE_CURRENT_POSITION_HPP
#define EDYN_SYS_UPDATE_CURRENT_POSITION_HPP

#include <entt/entt.hpp>
#include "edyn/comp/position.hpp"
#include "edyn/comp/current_position.hpp"
#include "edyn/comp/linvel.hpp"

namespace edyn {

void update_current_position(entt::registry &registry, scalar fixed_dt, scalar residual_dt) {
    const auto dt = residual_dt - fixed_dt;
    auto view = registry.view<current_position, const position, const linvel>();
    view.each([&] (auto, current_position &currpos, const position &pos, const linvel &vel) {
        currpos = pos + vel * dt;
    });
}

}

#endif // EDYN_SYS_UPDATE_CURRENT_POSITION_HPP