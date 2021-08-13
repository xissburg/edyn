#ifndef EDYN_SYS_UPDATE_INERTIA_HPP
#define EDYN_SYS_UPDATE_INERTIA_HPP

#include <entt/entity/fwd.hpp>

namespace edyn {

/**
 * @brief Updates all `inertia_world_inv` by rotating their inertia tensor
 * to world space.
 */
void update_inertias(entt::registry &);

/**
 * @brief Updates a single `inertia_world_inv`.
 * @param registry Data source.
 * @param entity Entity to be updated.
 */
void update_inertia(entt::registry &registry, entt::entity entity);

}

#endif // EDYN_SYS_UPDATE_INERTIA_HPP
