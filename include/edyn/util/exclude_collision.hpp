#ifndef EDYN_UTIL_EXCLUDE_COLLISION_HPP
#define EDYN_UTIL_EXCLUDE_COLLISION_HPP

#include <entt/entity/fwd.hpp>
#include "edyn/util/entity_pair.hpp"

namespace edyn {

/**
 * @brief Excludes collisions between a pair of entities.
 * Use this when collision filters are not enough.
 * @param registry Data source.
 * @param first The entity that should not collide with `second`.
 * @param second The entity that should not collide with `first`.
 */
void exclude_collision(entt::registry &registry, entt::entity first, entt::entity second);

/*! @copydoc exclude_collision */
void exclude_collision(entt::registry &registry, entity_pair entities);

}

#endif // EDYN_UTIL_EXCLUDE_COLLISION_HPP
