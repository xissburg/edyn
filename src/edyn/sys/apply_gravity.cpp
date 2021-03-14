#include "edyn/sys/apply_gravity.hpp"
#include "edyn/comp/gravity.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/math/constants.hpp"
#include <entt/entt.hpp>

namespace edyn {

void apply_gravity(entt::registry &registry, scalar dt) {
    auto gravity_view = registry.view<gravity>();
    auto inner_view = registry.view<position, mass, linvel>();

    gravity_view.each([&] (gravity &g) {
        auto [posA, mA, linvelA] = inner_view.get<position, mass, linvel>(g.body[0]);
        auto [posB, mB, linvelB] = inner_view.get<position, mass, linvel>(g.body[1]);

        auto d = posA - posB;
        auto l2 = length_sqr(d);
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
