#ifndef EDYN_SYS_INTEGRATE_SPIN_HPP
#define EDYN_SYS_INTEGRATE_SPIN_HPP

#include <entt/entt.hpp>
#include "edyn/math/math.hpp"
#include "edyn/comp/spin.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/dynamics/island_util.hpp"

namespace edyn {

inline void integrate_spin(entt::registry &registry, scalar dt) {
    auto view = registry.view<dynamic_tag, spin_angle, const spin>(exclude_global);
    view.each([&] (auto, auto, spin_angle &a, const spin &s) {
        // Keep angle in [0, 2π) interval by separating it into a complete turn
        // count (2π rad) and an angle in that interval.
        a += s * dt;
        auto spin_count = std::floor(a / pi2);
        a.count += spin_count;
        a -= spin_count * pi2;
    });
}

}

#endif // EDYN_SYS_INTEGRATE_SPIN_HPP