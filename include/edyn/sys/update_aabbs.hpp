#ifndef EDYN_SYS_UPDATE_AABBS_HPP
#define EDYN_SYS_UPDATE_AABBS_HPP

#include <entt/entity/fwd.hpp>
#include <vector>

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
 * @brief Update AABBs of given entities.
 * @remark This must be called after transforms are changed manually.
 * @param registry Data source.
 * @param entities Entities to be updated.
 */
void update_aabbs(entt::registry &registry, const entt::sparse_set &entities);

/*! @copydoc update_aabbs */
void update_aabbs(entt::registry &registry, const std::vector<entt::entity> &entities);

/**
 * @brief Update AABB of a single entity.
 * @param registry The registry to be updated.
 * @param entity Entity to be updated.
 */
void update_aabb(entt::registry &registry, entt::entity entity);

void update_island_aabbs(entt::registry &registry);
void update_island_aabbs(entt::registry &registry, const entt::sparse_set &entities);
void update_island_aabbs(entt::registry &registry, const std::vector<entt::entity> &entities);

}

#endif // EDYN_SYS_UPDATE_AABBS_HPP
