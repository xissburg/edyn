#ifndef EDYN_DYNAMICS_ISLAND_UTIL_HPP
#define EDYN_DYNAMICS_ISLAND_UTIL_HPP

#include <entt/fwd.hpp>
#include "edyn/comp/relation.hpp"
#include "edyn/math/scalar.hpp"
#include "edyn/comp/tag.hpp"

namespace edyn {

/**
 * Global exclusion list which covers the range of entities that should not
 * take part in any step of the simulation update, e.g. sleeping and disabled
 * entities.
 */
constexpr auto exclude_global = entt::exclude_t<sleeping_tag, disabled_tag>{};

/**
 * Wake up one entity. It wakes up the island where the entity is in.
 */
void wakeup(entt::entity, entt::registry &);

/**
 * Wake up one island.
 */
void wakeup_island(entt::entity, entt::registry &);

void put_islands_to_sleep(entt::registry &, uint64_t step, scalar dt);

}

#endif // EDYN_DYNAMICS_ISLAND_UTIL_HPP