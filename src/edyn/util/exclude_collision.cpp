#include "edyn/util/exclude_collision.hpp"
#include "edyn/comp/collision_exclusion.hpp"
#include "edyn/config/config.h"
#include <entt/entity/registry.hpp>

namespace edyn {

static
void exclude_collision_one_way(entt::registry &registry, entt::entity first, entt::entity second) {
    auto &exclusion = registry.get_or_emplace<collision_exclusion>(first);
    EDYN_ASSERT(exclusion.num_entities() + 1 < collision_exclusion::max_exclusions);
    exclusion.entity[exclusion.num_entities()] = second;
}

void exclude_collision(entt::registry &registry, entt::entity first, entt::entity second) {
    exclude_collision_one_way(registry, first, second);
    exclude_collision_one_way(registry, second, first);
}

void exclude_collision(entt::registry &registry, entity_pair entities) {
    exclude_collision(registry, entities.first, entities.second);
}

}