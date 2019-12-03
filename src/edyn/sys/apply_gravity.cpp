#include "edyn/sys/apply_gravity.hpp"
#include "edyn/comp/gravity.hpp"
#include "edyn/comp/relation.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/dynamics/island_util.hpp"
#include <entt/entt.hpp>

namespace edyn {

void apply_gravity(entt::registry &registry, scalar dt) {
    auto view = registry.view<relation, gravity>(exclude_sleeping);
    auto inner_view = registry.view<const position, const mass, linvel>(exclude_sleeping);

    view.each([&] (auto, relation &rel, auto g) {
        auto [posA, mA, linvelA] = inner_view.get<const position, const mass, linvel>(rel.entity[0]);
        auto [posB, mB, linvelB] = inner_view.get<const position, const mass, linvel>(rel.entity[1]);
        
        auto d = posA - posB;
        auto l2 = length2(d);
        l2 = std::max(l2, EDYN_EPSILON);

        auto l = std::sqrt(l2);
        auto dn = d / l;

        auto F = gravitational_constant * mA * mB / l2;
        auto P = F * dt * dn;
        linvelA -= P / mA;
        linvelB += P / mB;
    });

    
}

}
