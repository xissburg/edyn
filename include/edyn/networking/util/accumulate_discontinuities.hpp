#ifndef EDYN_NETWORKING_UTIL_ACCUMULATE_DISCONTINUITIES_HPP
#define EDYN_NETWORKING_UTIL_ACCUMULATE_DISCONTINUITIES_HPP

#include <entt/entity/registry.hpp>
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/networking/comp/discontinuity.hpp"

namespace edyn {

inline void accumulate_discontinuities(entt::registry &registry) {
    auto discontinuity_view = registry.view<previous_position, position, previous_orientation, orientation, discontinuity>();

    for (auto [entity, p_pos, pos, p_orn, orn, discontinuity] : discontinuity_view.each()) {
        discontinuity.position_offset += p_pos - pos;
        discontinuity.orientation_offset *= p_orn * conjugate(orn);
    }
}

}

#endif // EDYN_NETWORKING_UTIL_ACCUMULATE_DISCONTINUITIES_HPP
