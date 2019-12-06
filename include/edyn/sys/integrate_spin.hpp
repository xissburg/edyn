#ifndef EDYN_SYS_INTEGRATE_SPIN_HPP
#define EDYN_SYS_INTEGRATE_SPIN_HPP

#include <entt/entt.hpp>
#include "edyn/comp/spin.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/dynamics/island_util.hpp"

namespace edyn {

inline void integrate_spin(entt::registry &registry, scalar dt) {
    auto view = registry.view<dynamic_tag, spin_angle, const spin>(exclude_sleeping);
    view.each([&] (auto, auto, spin_angle &a, const spin &s) {
        a += s * dt;
    });
}

}

#endif // EDYN_SYS_INTEGRATE_SPIN_HPP