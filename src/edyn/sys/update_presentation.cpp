#include "edyn/sys/update_presentation.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/present_position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/present_orientation.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/context/settings.hpp"
#include "edyn/networking/comp/discontinuity.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

void update_presentation(entt::registry &registry, double sim_time, double time) {
    auto exclude = entt::exclude<sleeping_tag, disabled_tag>;
    auto linear_view = registry.view<position, linvel, present_position, procedural_tag>(exclude);
    auto angular_view = registry.view<orientation, angvel, present_orientation, procedural_tag>(exclude);
    const double fixed_dt = registry.ctx().at<settings>().fixed_dt;
    const auto dt = time - sim_time - fixed_dt;

    linear_view.each([dt](position &pos, linvel &vel, present_position &pre) {
        pre = pos + vel * dt;
    });

    angular_view.each([dt](orientation &orn, angvel &vel, present_orientation &pre) {
        pre = integrate(orn, vel, dt);
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
