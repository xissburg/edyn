#include "edyn/sys/update_presentation.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/present_position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/present_orientation.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/spin.hpp"
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
    auto &settings = registry.ctx().at<edyn::settings>();
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

    // Deal with spin.
    registry.view<discontinuity_spin, discontinuity_spin_accumulator>()
        .each([](discontinuity_spin &dis, discontinuity_spin_accumulator &accum) {
            dis.offset += accum.offset;
            accum.offset = 0;
        });

    registry.view<discontinuity_spin>()
        .each([rate, dt](discontinuity_spin &dis) {
            if (std::abs(dis.offset) > scalar(0.0001)) {
                dis.offset -= dis.offset * std::min(rate * dt, 1.0);
            } else {
                dis.offset = 0;
            }
        });

    registry.view<discontinuity_spin, sleeping_tag>()
        .each([](discontinuity_spin &dis) {
            dis.offset = 0;
        });
}

void update_presentation(entt::registry &registry, double sim_time, double current_time,
                         double delta_time, double presentation_delay) {
    auto &settings = registry.ctx().at<edyn::settings>();

    if (std::holds_alternative<client_network_settings>(settings.network_settings)) {
        update_discontinuities(registry, delta_time);
    }

    auto linear_view = registry.view<position, linvel, present_position, procedural_tag>(exclude_sleeping_disabled);
    auto angular_view = registry.view<orientation, angvel, present_orientation, procedural_tag>(exclude_sleeping_disabled);

    // Interpolate transforms at `sim_time` towards a consistent point in time
    // which is `presentation_delay` seconds behind the current time.
    const auto interpolation_dt = current_time - presentation_delay - sim_time;

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

    // Deal with spin.
    auto spin_view = registry.view<spin_angle, spin, present_spin_angle, procedural_tag>(exclude_sleeping_disabled);

    spin_view.each([interpolation_dt] (spin_angle &angle, spin &spin, present_spin_angle &pre) {
        pre.s = angle.s + spin.s * interpolation_dt;
    });

    for (auto [e, dis, p_spin] : registry.view<discontinuity_spin, present_spin_angle>().each()) {
        p_spin.s += dis.offset;
    }
}

void snap_presentation(entt::registry &registry) {
    auto view = registry.view<position, orientation, present_position, present_orientation>();
    view.each([](position &pos, orientation &orn, present_position &p_pos, present_orientation &p_orn) {
        p_pos = pos;
        p_orn = orn;
    });

    registry.view<spin_angle, present_spin_angle>().each([](spin_angle &angle, present_spin_angle &p_angle) {
        p_angle.s = angle.s;
    });
}

}
