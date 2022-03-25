#include "edyn/sys/update_presentation.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/present_position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/present_orientation.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/context/settings.hpp"
#include "edyn/networking/comp/discontinuity.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

void update_presentation(entt::registry &registry, double time) {
    auto timestamp_view = registry.view<island_timestamp>();
    auto exclude = entt::exclude<sleeping_tag, disabled_tag>;
    auto linear_view = registry.view<position, linvel, present_position, island_resident, procedural_tag>(exclude);
    auto angular_view = registry.view<orientation, angvel, present_orientation, island_resident, procedural_tag>(exclude);
    auto fixed_dt = registry.ctx<settings>().fixed_dt;

    linear_view.each([&] (position &pos, linvel &vel, present_position &pre, island_resident &resident) {
        EDYN_ASSERT(registry.valid(resident.island_entity));
        auto &isle_time = timestamp_view.get<island_timestamp>(resident.island_entity);
        EDYN_ASSERT(!(time < isle_time.value));
        auto dt = std::min(scalar(time - fixed_dt - isle_time.value), fixed_dt);
        pre = pos + vel * dt;
    });

    angular_view.each([&] (orientation &orn, angvel &vel, present_orientation &pre, island_resident &resident) {
        EDYN_ASSERT(registry.valid(resident.island_entity));
        auto &isle_time = timestamp_view.get<island_timestamp>(resident.island_entity);
        EDYN_ASSERT(!(time < isle_time.value));
        auto dt = std::min(scalar(time - fixed_dt - isle_time.value), fixed_dt);
        pre = integrate(orn, vel, dt);
    });

    auto discontinuity_view = registry.view<discontinuity, present_position, present_orientation>();
    discontinuity_view.each([] (discontinuity &dis, present_position &p_pos, present_orientation &p_orn) {
        p_pos += dis.position_offset;
        p_orn = dis.orientation_offset * p_orn;
    });
}

void snap_presentation(entt::registry &registry) {
    auto view = registry.view<position, orientation, present_position, present_orientation>();
    view.each([] (position &pos, orientation &orn, present_position &p_pos, present_orientation &p_orn) {
        p_pos = pos;
        p_orn = orn;
    });
}

}
