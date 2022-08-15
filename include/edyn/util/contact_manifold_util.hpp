#ifndef EDYN_UTIL_CONTACT_MANIFOLD_UTIL_HPP
#define EDYN_UTIL_CONTACT_MANIFOLD_UTIL_HPP

#include "edyn/core/entity_pair.hpp"
#include <entt/entity/fwd.hpp>

namespace edyn {

/**
 * @brief Checks whether there is a contact manifold connecting the two entities.
 * @param registry Data source.
 * @param first One entity.
 * @param second Another entity.
 * @return Whether a contact manifold exists between the two entities.
 */
bool manifold_exists(entt::registry &registry, entt::entity first, entt::entity second);

/*! @copydoc manifold_exists */
bool manifold_exists(entt::registry &registry, entity_pair entities);

/**
 * @brief Get contact manifold entity for a pair of entities.
 * Asserts if the manifold does not exist.
 * @param registry Data source.
 * @param first One entity.
 * @param second Another entity.
 * @return Contact manifold entity.
 */
entt::entity get_manifold_entity(const entt::registry &registry, entt::entity first, entt::entity second);

/*! @copydoc get_manifold_entity */
entt::entity get_manifold_entity(const entt::registry &registry, entity_pair entities);

}

#endif // EDYN_UTIL_CONTACT_MANIFOLD_UTIL_HPP
