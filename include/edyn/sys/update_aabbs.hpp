#ifndef EDYN_SYS_UPDATE_AABBS_HPP
#define EDYN_SYS_UPDATE_AABBS_HPP

#include <entt/entity/fwd.hpp>

namespace edyn {

/**
 * @brief Update AABBs of all entities that contain a shape.
 * @remark It's important to call this after the rotated meshes of all
 * polyhedrons are updated because they will be used to calculate the AABBs of
 * polyhedrons.
 * @param registry The registry to be updated.
 */
void update_aabbs(entt::registry &registry);

/**
 * @brief Update AABB of a single entity.
 * @param registry The registry to be updated.
 * @param entity Entity to be updated.
 */
void update_aabb(entt::registry &registry, entt::entity entity);

void update_island_aabbs(entt::registry &registry);

}

#endif // EDYN_SYS_UPDATE_AABBS_HPP
