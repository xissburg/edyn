#ifndef EDYN_SYS_INTEGRATE_LINACC_HPP
#define EDYN_SYS_INTEGRATE_LINACC_HPP

#include <entt/entt.hpp>
#include "../comp/linvel.hpp"
#include "../comp/linacc.hpp"

namespace edyn {

void integrate_linacc(entt::registry& registry, scalar dt) {
    auto view = registry.view<edyn::linvel, const edyn::linacc>();
    view.each([&] (auto, linvel& vel, const linacc& acc) {
        vel += acc * dt;
    });
}

}

#endif // EDYN_SYS_INTEGRATE_LINACC_HPP