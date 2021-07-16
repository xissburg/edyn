#include "edyn/collision/should_collide.hpp"
#include "edyn/comp/collision_filter.hpp"
#include "edyn/comp/collision_exclusion.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

static
bool should_exclude(const entt::registry &registry, entt::entity first, entt::entity second) {
    if (auto *exclusion = registry.try_get<collision_exclusion>(first)) {
        for (unsigned i = 0; i < exclusion->num_entities; ++i) {
            if (exclusion->entity[i] == second) {
                return true;
            }
        }
    }

    return false;
}

bool should_collide_default(const entt::registry &registry, entt::entity first, entt::entity second) {
    if (first == second) {
        return false;
    }

    auto view = registry.view<collision_filter>();
    auto &filter0 = view.get(first);
    auto &filter1 = view.get(second);

    if ((filter0.group & filter1.mask) == 0 ||
        (filter1.group & filter0.mask) == 0) {
        return false;
    }

    if (should_exclude(registry, first, second) ||
        should_exclude(registry, second, first)) {
        return false;
    }

    return true;
}

}
