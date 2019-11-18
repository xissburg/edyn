#ifndef EDYN_COMP_CONSTRAINT_HPP
#define EDYN_COMP_CONSTRAINT_HPP

#include <variant>
#include "edyn/constraints/distance_constraint.hpp"
#include "edyn/constraints/point_constraint.hpp"

namespace edyn {

struct constraint {
    std::array<entt::entity, 2> entity;
    std::array<entt::entity, 8> row;

    std::variant<distance_constraint> var;
};

}

#endif // EDYN_COMP_CONSTRAINT_HPP