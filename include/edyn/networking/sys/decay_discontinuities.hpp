#ifndef EDYN_NETWORKING_SYS_DECAY_DISCONTINUITIES_HPP
#define EDYN_NETWORKING_SYS_DECAY_DISCONTINUITIES_HPP

#include <entt/entity/registry.hpp>
#include "edyn/context/settings.hpp"
#include "edyn/math/scalar.hpp"
#include "edyn/networking/comp/discontinuity.hpp"

namespace edyn {

inline void decay_discontinuities(entt::registry &registry, scalar rate) {
    EDYN_ASSERT(!(rate < 0) && rate < 1);
    registry.view<discontinuity>().each([rate](discontinuity &dis) {
        dis.position_offset *= rate;
        dis.orientation_offset = slerp(quaternion_identity, dis.orientation_offset, rate);
    });
}

inline void decay_discontinuities(entt::registry &registry) {
    auto &settings = registry.ctx().at<edyn::settings>();

    if (std::holds_alternative<client_network_settings>(settings.network_settings)) {
        auto &network_settings = std::get<client_network_settings>(settings.network_settings);
        decay_discontinuities(registry, network_settings.discontinuity_decay_rate);
    }
}

}

#endif // EDYN_NETWORKING_SYS_DECAY_DISCONTINUITIES_HPP
