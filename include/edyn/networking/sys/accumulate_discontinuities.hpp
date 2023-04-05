#ifndef EDYN_NETWORKING_SYS_ACCUMULATE_DISCONTINUITIES_HPP
#define EDYN_NETWORKING_SYS_ACCUMULATE_DISCONTINUITIES_HPP

#include <entt/entity/registry.hpp>
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/networking/comp/discontinuity.hpp"
#include "edyn/util/island_util.hpp"

namespace edyn {

inline void accumulate_discontinuities(entt::registry &registry) {
    auto accum_view = registry.view<previous_position, position,
                                    previous_orientation, orientation,
                                    discontinuity_accumulator>(exclude_sleeping_disabled);

    for (auto [entity, p_pos, pos, p_orn, orn, accum] : accum_view.each()) {
        accum.position_offset += p_pos - pos;
        accum.orientation_offset *= p_orn * conjugate(orn);
        registry.patch<discontinuity_accumulator>(entity);
    }
}

inline void clear_accumulated_discontinuities_quietly(entt::registry &registry) {
    auto accum_view = registry.view<discontinuity_accumulator>(exclude_sleeping_disabled);
    accum_view.each([](discontinuity_accumulator &accum) {
        accum.position_offset = edyn::vector3_zero;
        accum.orientation_offset = edyn::quaternion_identity;
    });
}

}

#endif // EDYN_NETWORKING_SYS_ACCUMULATE_DISCONTINUITIES_HPP
