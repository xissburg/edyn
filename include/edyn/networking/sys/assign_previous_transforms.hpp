#ifndef EDYN_NETWORKING_SYS_ASSIGN_PREVIOUS_TRANSFORMS_HPP
#define EDYN_NETWORKING_SYS_ASSIGN_PREVIOUS_TRANSFORMS_HPP

#include <entt/entity/registry.hpp>
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/networking/comp/discontinuity.hpp"

namespace edyn {

inline void assign_previous_transforms(entt::registry &registry) {
    registry.view<previous_position, position>().each([](previous_position &p_pos, position &pos) {
        p_pos = pos;
    });

    registry.view<previous_orientation, orientation>().each([](previous_orientation &p_orn, orientation &orn) {
        p_orn = orn;
    });
}

}

#endif // EDYN_NETWORKING_SYS_ASSIGN_PREVIOUS_TRANSFORMS_HPP
