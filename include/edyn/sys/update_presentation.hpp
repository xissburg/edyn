#ifndef EDYN_SYS_UPDATE_PRESENTATION_HPP
#define EDYN_SYS_UPDATE_PRESENTATION_HPP

#include <entt/entity/registry.hpp>
#include "edyn/comp/position.hpp"
#include "edyn/comp/present_position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/present_orientation.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/comp/tag.hpp"

namespace edyn {

inline void update_presentation(entt::registry &registry, double time) {
    auto timestamp_view = registry.view<island_timestamp>();
    auto exclude = entt::exclude<sleeping_tag, disabled_tag>;
    auto linear_view = registry.view<position, linvel, present_position, island_resident, procedural_tag>(exclude);
    auto angular_view = registry.view<orientation, angvel, present_orientation, island_resident, procedural_tag>(exclude);
    constexpr double max_dt = 0.02;

    linear_view.each([&] (position &pos, linvel &vel, present_position &pre, island_resident &resident) {
        EDYN_ASSERT(registry.valid(resident.island_entity));
        auto &isle_time = timestamp_view.get(resident.island_entity);
        auto dt = scalar(std::min(time - isle_time.value, max_dt));
        pre = pos + vel * dt;
    });

    angular_view.each([&] (orientation &orn, angvel &vel, present_orientation &pre, island_resident &resident) {
        EDYN_ASSERT(registry.valid(resident.island_entity));
        auto &isle_time = timestamp_view.get(resident.island_entity);
        auto dt = scalar(std::min(time - isle_time.value, max_dt));
        pre = integrate(orn, vel, dt);
    });
}

inline void snap_presentation(entt::registry &registry) {
    auto view = registry.view<position, orientation, present_position, present_orientation>();
    view.each([] (position &pos, orientation &orn, present_position &p_pos, present_orientation &p_orn) {
        p_pos = pos;
        p_orn = orn;
    });
}

}

#endif // EDYN_SYS_UPDATE_PRESENTATION_HPP