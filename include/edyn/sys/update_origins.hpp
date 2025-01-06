#ifndef EDYN_SYS_UPDATE_ORIGINS_HPP
#define EDYN_SYS_UPDATE_ORIGINS_HPP

#include <entt/entity/fwd.hpp>
#include <vector>

namespace edyn {

/**
 * @brief Recalculates cached origin of bodies that have a non-zero
 * center of mass offset.
 * @param registry Data source.
 */
void update_origins(entt::registry &);


/**
 * @brief Recalculates cached origin of given entities.
 * @param registry Data source.
 * @param entities Entities to be updated.
 */
void update_origins(entt::registry &, const entt::sparse_set &);

/*! @copydoc update_origins */
void update_origins(entt::registry &, const std::vector<entt::entity> &);

}

#endif // EDYN_SYS_UPDATE_ORIGINS_HPP
