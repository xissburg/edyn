#ifndef EDYN_COMP_ISLAND_HPP
#define EDYN_COMP_ISLAND_HPP

#include <limits>
#include <optional>
#include <unordered_set>
#include <entt/entity/fwd.hpp>
#include <entt/entity/entity.hpp>
#include <entt/entity/sparse_set.hpp>
#include "edyn/comp/aabb.hpp"

namespace edyn {

/**
 * @brief An _island_ is a set of entities that can affect one another,
 * usually through constraints.
 */
struct island {
    entt::sparse_set nodes {};
    entt::sparse_set edges {};
    std::optional<double> sleep_timestamp;
};

struct island_AABB : public AABB {};

/**
 * @brief Component assigned to an entity that resides in an island, i.e.
 * procedural entities which can only be present in a single island.
 */
struct island_resident {
    entt::entity island_entity {entt::null};
};

/**
 * @brief Component assigned to an entity that resides in multiple islands,
 * i.e. non-procedural entities which can be present in multiple islands
 * simultaneously.
 */
struct multi_island_resident {
    entt::sparse_set island_entities {};
};

}

#endif // EDYN_COMP_ISLAND_HPP
