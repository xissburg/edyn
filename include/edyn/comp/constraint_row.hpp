#ifndef EDYN_COMP_CONSTRAINT_ROW_HPP
#define EDYN_COMP_CONSTRAINT_ROW_HPP

#include <array>
#include <entt/fwd.hpp>
#include "relation.hpp"
#include "edyn/math/vector3.hpp"

namespace edyn {

struct constraint_row {
    std::array<entt::entity, max_relations> entity;
    std::array<vector3, 4> J;
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
};

}

#endif // EDYN_COMP_CONSTRAINT_ROW_HPP