#ifndef EDYN_COMP_CONSTRAINT_ROW_HPP
#define EDYN_COMP_CONSTRAINT_ROW_HPP

#include <array>
#include <entt/fwd.hpp>
#include "relation.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/util/array.hpp"

namespace edyn {

struct constraint_row {
    std::array<entt::entity, max_relations> entity {make_array<max_relations>(entt::entity{entt::null})};
    std::array<vector3, max_relations * 2> J;
    scalar error;
    scalar lower_limit;
    scalar upper_limit;
    // Effective mass (J M^-1 J^T)^-1
    scalar eff_mass;
    // Right hand side Jv + b
    scalar rhs;
    scalar relvel;
    scalar restitution {0};
    scalar impulse {0};
    int priority {0};
};

}

#endif // EDYN_COMP_CONSTRAINT_ROW_HPP