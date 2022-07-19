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

struct island_stats {
    unsigned num_nodes {0};
    unsigned num_edges {0};

    auto size() const {
        return num_nodes + num_edges;
    }
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

using island_worker_index_type = uint16_t;
static constexpr auto invalid_worker_index = std::numeric_limits<island_worker_index_type>::max();

struct island_worker_resident {
    island_worker_index_type worker_index {invalid_worker_index};
};

struct multi_island_worker_resident {
    std::unordered_set<island_worker_index_type> worker_indices;
};

}

#endif // EDYN_COMP_ISLAND_HPP
