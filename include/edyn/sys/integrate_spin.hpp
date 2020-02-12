#ifndef EDYN_SYS_INTEGRATE_SPIN_HPP
#define EDYN_SYS_INTEGRATE_SPIN_HPP

#include <entt/entt.hpp>
#include "edyn/math/math.hpp"
#include "edyn/comp/spin.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/dynamics/island_util.hpp"

namespace edyn {

inline void integrate_spin(entt::registry &registry, scalar dt) {
    auto view = registry.view<dynamic_tag, spin_angle, const spin>(exclude_sleeping);
    view.each([&] (auto, auto, spin_angle &a, const spin &s) {
        auto angle_delta = s * dt;
        auto accum_angle = a.accum_angle + angle_delta;
        auto spin_count = std::floor(accum_angle / pi2);
        a.count += spin_count;
        a.accum_angle = accum_angle - spin_count * pi2;
        a.s = normalize_angle(a.s + angle_delta);
    });
}

}

#endif // EDYN_SYS_INTEGRATE_SPIN_HPP