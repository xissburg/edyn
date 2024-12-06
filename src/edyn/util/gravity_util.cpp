#include "edyn/util/gravity_util.hpp"
#include "edyn/comp/gravity.hpp"
#include "edyn/context/settings.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

vector3 get_gravity(const entt::registry &registry) {
    return registry.ctx().get<settings>().gravity;
}

void set_gravity(entt::registry &registry, vector3 gravity) {
    registry.ctx().get<settings>().gravity = gravity;

    auto view = registry.view<edyn::gravity>();

    for (auto entity : view) {
        registry.replace<edyn::gravity>(entity, gravity);
    }
}

}
