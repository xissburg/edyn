#ifndef EDYN_COMP_RELATION_HPP
#define EDYN_COMP_RELATION_HPP

#include <entt/entt.hpp>
#include "edyn/util/array.hpp"

namespace edyn {

inline constexpr size_t max_relations = 2;

/**
 * @brief A relation between entities ensures that they'll be kept in the same
 * island and thus in the same solver. Components that operate on a set of
 * entities (e.g. `edyn::constraint`) require a relation to exist as well.
 */
struct relation {
    std::array<entt::entity, max_relations> entity =
        make_array<max_relations>(entt::entity{entt::null});
};

/**
 * @brief A _relation container_ keeps a list of entities that contain a
 * `edyn::relation` which refers to the entity this component is assigned to.
 * This allows an entity to quickly obtain the list of relations it is
 * involved with.
 */
struct relation_container {
    std::vector<entt::entity> entities;
};

}

#endif // EDYN_COMP_RELATION_HPP