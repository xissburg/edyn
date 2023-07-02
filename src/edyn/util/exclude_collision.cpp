#include "edyn/util/exclude_collision.hpp"
#include "edyn/comp/collision_exclusion.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/config/config.h"
#include <entt/entity/registry.hpp>

namespace edyn {

static
void exclude_collision_one_way(entt::registry &registry, entt::entity first, entt::entity second) {
    EDYN_ASSERT(registry.all_of<rigidbody_tag>(first));
    if (!registry.all_of<collision_exclusion>(first)) {
        registry.emplace<collision_exclusion>(first);
    }

    auto &exclusion = registry.get<collision_exclusion>(first);
    const auto size = exclusion.num_entities();

    for (unsigned i = 0; i < size; ++i) {
        if (exclusion.entity[i] == second) {
            return;
        }
    }

    EDYN_ASSERT(size < collision_exclusion::max_exclusions);
    exclusion.entity[size] = second;
    registry.patch<collision_exclusion>(first);
}

void exclude_collision(entt::registry &registry, entt::entity first, entt::entity second) {
    exclude_collision_one_way(registry, first, second);
    exclude_collision_one_way(registry, second, first);
}

void exclude_collision(entt::registry &registry, entity_pair entities) {
    exclude_collision(registry, entities.first, entities.second);
}

void remove_collision_exclusion_one_way(entt::registry &registry, entt::entity first, entt::entity second) {
    registry.patch<collision_exclusion>(first,
        [&](collision_exclusion &exclusion) {
            const auto size = exclusion.num_entities();
            for (auto i = size; i; --i) {
                const auto k = i - 1;
                if (exclusion.entity[k] == second) {
                    exclusion.entity[k] = exclusion.entity[size - 1];
                    exclusion.entity[size - 1] = entt::null;
                    break;
                }
            }
        });
}

void remove_collision_exclusion(entt::registry &registry, entt::entity first, entt::entity second) {
    remove_collision_exclusion_one_way(registry, first, second);
    remove_collision_exclusion_one_way(registry, second, first);
}

void clear_collision_exclusion(entt::registry &registry, entt::entity entity) {
    auto &exclude = registry.get<collision_exclusion>(entity);
    const auto size = exclude.num_entities();

    for (unsigned i = 0; i < size; ++i) {
        auto other = exclude.entity[i];
        remove_collision_exclusion_one_way(registry, other, entity);
    }

    registry.remove<collision_exclusion>(entity);
}

}
