#ifndef EDYN_SYS_INTEGRATE_GRAVITY_HPP
#define EDYN_SYS_INTEGRATE_GRAVITY_HPP

#include <entt/entt.hpp>
#include "edyn/comp/mass.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/gravity.hpp"
#include "edyn/math/constants.hpp"

namespace edyn {

void integrate_gravity(entt::registry &registry, scalar dt) {
    auto view = registry.view<const position, linvel, const mass, const gravity>();

    // Apply pair-wise gravity attraction force. Complexity O(n^2).
    for (auto it1 = view.begin(); it1 + 1 != view.end(); ++it1) {
        auto cmp1 = view.get<const position, linvel, const mass, const gravity>(*it1);
        const auto& pos1 = std::get<const position&>(cmp1);
        const auto& m1 = std::get<const mass&>(cmp1);
        auto& vel1 = std::get<linvel&>(cmp1);

        for (auto it2 = it1 + 1; it2 != view.end(); ++it2) {
            auto cmp2 = view.get<const position, linvel, const mass, const gravity>(*it2);
            const auto& pos2 = std::get<const position&>(cmp2);
            const auto& m2 = std::get<const mass&>(cmp2);
            auto& vel2 = std::get<linvel&>(cmp2);

            auto d = pos1 - pos2;
            auto l2 = std::max(edyn::length2(d), EDYN_EPSILON);
            auto l = std::sqrt(l2);
            auto dn = d / l;

            auto F = dn * (gravitational_constant * m1 * m2 / l2);

            auto dv2 =  F / m2 * dt;
            vel2 += dv2;

            auto dv1 =  F / m1 * dt;
            vel1 -= dv1;
        }
    }
}

}

#endif // EDYN_SYS_INTEGRATE_GRAVITY_HPP