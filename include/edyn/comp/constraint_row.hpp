#ifndef EDYN_COMP_CONSTRAINT_ROW_HPP
#define EDYN_COMP_CONSTRAINT_ROW_HPP

#include <array>
#include <entt/fwd.hpp>
#include "edyn/math/vector3.hpp"

namespace edyn {

struct constraint_row {
    entt::entity parent;
    std::array<vector3, 4> J;
    scalar error;
    scalar lower_limit;
    scalar upper_limit;
    // Effective mass (J M^-1 J^T)^-1
    scalar eff_mass;
    // Right hand side Jv + b
    scalar rhs;
    scalar impulse;
};

}

#endif // EDYN_COMP_CONSTRAINT_ROW_HPP