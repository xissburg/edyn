#ifndef EDYN_UTIL_EXCLUDE_COLLISION_HPP
#define EDYN_UTIL_EXCLUDE_COLLISION_HPP

#include <entt/entity/fwd.hpp>
#include "edyn/core/entity_pair.hpp"

namespace edyn {

/**
 * @brief Excludes collisions between a pair of entities.
 * Use this when collision filters are not enough.
 * @remark Must be removed with `clear_collision_exclusion` when the entity is
 * destroyed to avoid having an invalid entity in the exclusion list of others.
 * This won't cause UB but will occupy valuable space in the fixed-size array
 * of exclusions.
 * @param registry Data source.
 * @param first The entity that should not collide with `second`.
 * @param second The entity that should not collide with `first`.
 */
void exclude_collision(entt::registry &registry, entt::entity first, entt::entity second);

/*! @copydoc exclude_collision */
void exclude_collision(entt::registry &registry, entity_pair entities);

/**
 * @brief Remove a collision exclusion between two entities.
 * @param registry Data source.
 * @param first Entity that has an exclusion with `second`.
 * @param second Entity that has an exclusion with `first`.
 */
void remove_collision_exclusion(entt::registry &registry, entt::entity first, entt::entity second);

/**
 * @brief Remove all collision exclusions for the given entity.
 * @param registry Data source.
 * @param entity Entity to be cleared of exclusions.
 */
void clear_collision_exclusion(entt::registry &registry, entt::entity entity);

}

#endif // EDYN_UTIL_EXCLUDE_COLLISION_HPP
