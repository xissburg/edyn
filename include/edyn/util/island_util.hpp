#ifndef EDYN_DYNAMICS_ISLAND_UTIL_HPP
#define EDYN_DYNAMICS_ISLAND_UTIL_HPP

#include <entt/fwd.hpp>
#include <entt/entity/utility.hpp>
#include "edyn/math/scalar.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/util/entity_set.hpp"

namespace edyn {

/**
 * Returns all nodes in the `node_entity` sub-tree.
 */
entity_set get_island_node_children(const entt::registry &, entt::entity node_entity);

/**
 * Destroys children of a parent node that was destroyed.
 */
void on_destroy_island_node_parent(entt::registry &, entt::entity);

}

#endif // EDYN_DYNAMICS_ISLAND_UTIL_HPP