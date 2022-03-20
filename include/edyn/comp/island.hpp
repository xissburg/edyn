#ifndef EDYN_COMP_ISLAND_HPP
#define EDYN_COMP_ISLAND_HPP

#include <entt/entity/fwd.hpp>
#include <entt/entity/entity.hpp>
#include <entt/entity/sparse_set.hpp>

namespace edyn {

/**
 * @brief An _island_ is a set of entities that can affect one another,
 * usually through constraints.
 */
struct island {
    entt::sparse_set nodes {};
    entt::sparse_set edges {};
};

/**
 * @brief Timestamp of the current state of the simulation in an island.
 */
struct island_timestamp {
    double value;
};

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

template<typename Archive>
void serialize(Archive &archive, island_timestamp &timestamp) {
    archive(timestamp.value);
}

}

#endif // EDYN_COMP_ISLAND_HPP
