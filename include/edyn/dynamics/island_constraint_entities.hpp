#ifndef EDYN_DYNAMICS_ISLAND_CONSTRAINT_ENTITIES_HPP
#define EDYN_DYNAMICS_ISLAND_CONSTRAINT_ENTITIES_HPP

#include <array>
#include <vector>
#include "edyn/constraints/constraint.hpp"

namespace edyn {

struct island_constraint_entities {
    std::array<std::vector<entt::entity>, std::tuple_size_v<constraints_tuple_t>> entities;
};

}

#endif // EDYN_DYNAMICS_ISLAND_CONSTRAINT_ENTITIES_HPP
