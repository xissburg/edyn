#ifndef EDYN_CONSTRAINTS_CONSTRAINT_ROW_SPIN_FRICTION_HPP
#define EDYN_CONSTRAINTS_CONSTRAINT_ROW_SPIN_FRICTION_HPP

#include "edyn/constraints/constraint_row.hpp"
#include "edyn/math/vector3.hpp"
#include <vector>

namespace edyn {

struct constraint_row_spin_friction {
    // Include only angular components in Jacobian since this constraint only
    // applies angular impulses.
    std::array<vector3, 2> J;
    scalar eff_mass;
    scalar rhs;
    scalar impulse;
    scalar friction_coefficient;
    unsigned normal_row_index;
};

void solve_spin_friction(constraint_row_spin_friction &row, const std::vector<constraint_row> &row_cache);
void warm_start(constraint_row_spin_friction &row, const std::vector<constraint_row> &row_cache);

}

#endif // EDYN_CONSTRAINTS_CONSTRAINT_ROW_SPIN_FRICTION_HPP
