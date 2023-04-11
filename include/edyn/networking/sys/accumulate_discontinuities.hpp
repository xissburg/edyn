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

    auto discontinuity_spin_view = registry.view<previous_spin_angle, spin_angle, discontinuity_spin>();

    for (auto [e, p_spin, spin, dis] : discontinuity_spin_view.each()) {
        dis.offset += (p_spin.count - spin.count) * pi2 + p_spin.s - spin.s;
    }
}

inline void clear_accumulated_discontinuities_quietly(entt::registry &registry) {
    auto accum_view = registry.view<discontinuity_accumulator>(exclude_sleeping_disabled);
    accum_view.each([](discontinuity_accumulator &accum) {
        accum.position_offset = edyn::vector3_zero;
        accum.orientation_offset = edyn::quaternion_identity;
    });

    auto accum_spin_view = registry.view<discontinuity_spin_accumulator>();
    accum_spin_view.each([](discontinuity_spin_accumulator &accum) {
        accum.offset = 0;
    });
}

}

#endif // EDYN_NETWORKING_SYS_ACCUMULATE_DISCONTINUITIES_HPP
