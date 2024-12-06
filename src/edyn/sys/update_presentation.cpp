#include "edyn/sys/update_presentation.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/present_position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/present_orientation.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/context/settings.hpp"
#include "edyn/math/math.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/networking/comp/discontinuity.hpp"
#include "edyn/util/island_util.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

static void update_discontinuities(entt::registry &registry, double dt) {
    auto dis_view = registry.view<discontinuity>();
    auto accum_view = registry.view<discontinuity_accumulator>();

    // Transfer accumulated discontinuities.
    (dis_view | accum_view).each([](discontinuity &dis, discontinuity_accumulator &accum) {
        dis.position_offset += accum.position_offset;
        dis.orientation_offset = edyn::normalize(dis.orientation_offset * accum.orientation_offset);
        accum.position_offset = edyn::vector3_zero;
        accum.orientation_offset = edyn::quaternion_identity;
    });

    // Decay discontinuities.
    auto &settings = registry.ctx().get<edyn::settings>();
    auto &network_settings = std::get<client_network_settings>(settings.network_settings);
    const auto rate = network_settings.discontinuity_decay_rate;
    dis_view.each([rate, dt](discontinuity &dis) {
        if (length_sqr(dis.position_offset) > scalar(0.0001)) {
            dis.position_offset -= dis.position_offset * std::min(rate * dt, 1.0);
        } else {
            dis.position_offset = edyn::vector3_zero;
        }

        if (std::abs(dis.orientation_offset.w) < scalar(0.9999)) {
            dis.orientation_offset = slerp(dis.orientation_offset, quaternion_identity, std::min(rate * dt, 1.0));
        } else {
            dis.orientation_offset = quaternion_identity;
        }
    });

    auto sleeping_view = registry.view<sleeping_tag>();
    (dis_view | sleeping_view).each([](discontinuity &dis) {
        dis.position_offset = edyn::vector3_zero;
        dis.orientation_offset = quaternion_identity;
    });
}

void update_presentation(entt::registry &registry, double sim_time, double current_time,
                         double delta_time, double presentation_delay) {
    auto &settings = registry.ctx().get<edyn::settings>();

    if (std::holds_alternative<client_network_settings>(settings.network_settings)) {
        update_discontinuities(registry, delta_time);
    }

    auto linear_view = registry.view<position, linvel, present_position, procedural_tag>(exclude_sleeping_disabled);
    auto angular_view = registry.view<orientation, angvel, present_orientation, procedural_tag>(exclude_sleeping_disabled);

    // Interpolate transforms at `sim_time` towards a consistent point in time
    // which is `presentation_delay` seconds behind the current time.
    const auto interpolation_dt = std::min(static_cast<scalar>(current_time - presentation_delay - sim_time), settings.fixed_dt);

    linear_view.each([interpolation_dt](position &pos, linvel &vel, present_position &pre) {
        pre = pos + vel * interpolation_dt;
    });

    angular_view.each([interpolation_dt](orientation &orn, angvel &vel, present_orientation &pre) {
        pre = integrate(orn, vel, interpolation_dt);
    });

    auto discontinuity_view = registry.view<discontinuity, present_position, present_orientation>();
    discontinuity_view.each([](discontinuity &dis, present_position &p_pos, present_orientation &p_orn) {
        p_pos += dis.position_offset;
        p_orn = dis.orientation_offset * p_orn;
    });
}

void snap_presentation(entt::registry &registry) {
    auto view = registry.view<position, orientation, present_position, present_orientation>();
    view.each([](position &pos, orientation &orn, present_position &p_pos, present_orientation &p_orn) {
        p_pos = pos;
        p_orn = orn;
    });
}

}
