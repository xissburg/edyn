#ifndef EDYN_SYS_UPDATE_INERTIA_HPP
#define EDYN_SYS_UPDATE_INERTIA_HPP

#include <entt/entity/fwd.hpp>
#include <vector>

namespace edyn {

/**
 * @brief Updates all `inertia_world_inv` by rotating their inertia tensor
 * to world space.
 * @param registry Data source.
 */
void update_inertias(entt::registry &);

/**
 * @brief Updates a single `inertia_world_inv`.
 * @param registry Data source.
 * @param entity Entity to be updated.
 */
void update_inertia(entt::registry &registry, entt::entity entity);

/**
 * @brief Updates `inertia_world_inv` for the given entities.
 * @param registry Data source.
 * @param entities Entities to be updated.
 */
void update_inertias(entt::registry &registry, const entt::sparse_set &entities);

/*! @copydoc update_inertias */
void update_inertias(entt::registry &registry, const std::vector<entt::entity> &entities);

}

#endif // EDYN_SYS_UPDATE_INERTIA_HPP
