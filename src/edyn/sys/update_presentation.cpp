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
#include "edyn/networking/comp/discontinuity.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

void update_presentation(entt::registry &registry, double sim_time, double time) {
    auto exclude = entt::exclude<sleeping_tag, disabled_tag>;
    auto linear_view = registry.view<position, linvel, present_position, procedural_tag>(exclude);
    auto angular_view = registry.view<orientation, angvel, present_orientation, procedural_tag>(exclude);
    auto spin_view = registry.view<spin_angle, spin, present_spin_angle, procedural_tag>(exclude);
    const double fixed_dt = registry.ctx().at<settings>().fixed_dt;
    const auto dt = std::min(time - sim_time - fixed_dt, fixed_dt);

    linear_view.each([dt](position &pos, linvel &vel, present_position &pre) {
        pre = pos + vel * dt;
    });

    angular_view.each([dt](orientation &orn, angvel &vel, present_orientation &pre) {
        pre = integrate(orn, vel, dt);
    });

    spin_view.each([&] (spin_angle &angle, spin &spin, present_spin_angle &pre) {
        pre.s = angle.s + spin.s * dt;
    });

    auto discontinuity_view = registry.view<discontinuity, present_position, present_orientation>();
    discontinuity_view.each([](discontinuity &dis, present_position &p_pos, present_orientation &p_orn) {
        p_pos += dis.position_offset;
        p_orn = dis.orientation_offset * p_orn;
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
