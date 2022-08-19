#ifndef EDYN_DYNAMICS_ISLAND_CONSTRAINT_ENTITIES_HPP
#define EDYN_DYNAMICS_ISLAND_CONSTRAINT_ENTITIES_HPP

#include <array>
#include <vector>
#include "edyn/constraints/constraint.hpp"

namespace edyn {

/**
 * Stores the entities of each constraint type that reside in the island to which
 * this component is assigned. Each vector of entities is in direct correspondence
 * with the type in `constraints_tuple`. This component enables the solver to
 * go over constraint entities per constraint type.
 */
struct island_constraint_entities {
    std::array<std::vector<entt::entity>, std::tuple_size_v<constraints_tuple_t>> entities;
};

}

#endif // EDYN_DYNAMICS_ISLAND_CONSTRAINT_ENTITIES_HPP
