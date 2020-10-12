#ifndef EDYN_SYS_INTEGRATE_LINACC_HPP
#define EDYN_SYS_INTEGRATE_LINACC_HPP

#include <entt/entt.hpp>
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/linacc.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/dynamics/island_util.hpp"

namespace edyn {

inline void integrate_linacc(entt::registry &registry, scalar dt) {
    auto view = registry.view<linvel, linacc, dynamic_tag>(exclude_global);
    view.each([&] (entt::entity, linvel &vel, linacc &acc) {
        vel += acc * dt;
    });
}

}

#endif // EDYN_SYS_INTEGRATE_LINACC_HPP