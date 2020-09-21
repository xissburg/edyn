#ifndef EDYN_SYS_UPDATE_PRESENTATION_HPP
#define EDYN_SYS_UPDATE_PRESENTATION_HPP

#include <entt/entt.hpp>
#include "edyn/comp/position.hpp"
#include "edyn/comp/present_position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/present_orientation.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/dynamics/island_util.hpp"

namespace edyn {

inline void update_presentation(entt::registry &registry, double time) {
    auto island_view = registry.view<island>(exclude_global);
    island_view.each([time, &registry] (auto, island &isle) {
        auto dt = time - isle.timestamp;
        
        for (auto ent : isle.entities) {
            auto [pos, vel, pre] = registry.try_get<position, linvel, present_position>(ent);
            if (pos && vel && pre) {
                *pre = *pos + *vel * dt;
            }
        }

        for (auto ent : isle.entities) {
            auto [orn, vel, pre] = registry.try_get<orientation, angvel, present_orientation>(ent);
            if (orn && vel && pre) {
                *pre = integrate(*orn, *vel, dt);
            }
        }
    });
}

inline void snap_presentation(entt::registry &registry) {
    auto view = registry.view<position, orientation, present_position, present_orientation>();
    view.each([] (auto, position &pos, orientation &orn, present_position &p_pos, present_orientation &p_orn) {
        p_pos = pos;
        p_orn = orn;
    });
}

}

#endif // EDYN_SYS_UPDATE_PRESENTATION_HPP