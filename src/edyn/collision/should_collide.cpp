#include "edyn/collision/should_collide.hpp"
#include "edyn/comp/collision_filter.hpp"
#include <entt/entt.hpp>

namespace edyn {

bool should_collide_default(entt::registry &registry, entt::entity e0, entt::entity e1) {
    if (e0 == e1) {
        return false;
    }

    auto view = registry.view<collision_filter>();
    auto &filter0 = view.get(e0);
    auto &filter1 = view.get(e1);
    return (filter0.group & filter1.mask) != 0 &&
           (filter1.group & filter0.mask) != 0;
}

}
