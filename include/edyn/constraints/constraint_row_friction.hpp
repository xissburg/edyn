#ifndef EDYN_CONSTRAINTS_CONSTRAINT_ROW_FRICTION_HPP
#define EDYN_CONSTRAINTS_CONSTRAINT_ROW_FRICTION_HPP

#include "edyn/constraints/constraint_row.hpp"
#include "edyn/math/vector3.hpp"
#include <vector>

namespace edyn {

struct row_cache;

struct constraint_row_friction {
    struct individual_row {
        std::array<vector3, 4> J;
        scalar eff_mass;
        scalar rhs;
        scalar impulse;
    };

    individual_row row[2];
    scalar friction_coefficient;
    unsigned normal_row_index;
};

void solve_friction(constraint_row_friction &row, const std::vector<constraint_row> &row_cache);
void warm_start(constraint_row_friction &row, const std::vector<constraint_row> &row_cache);

}

#endif // EDYN_CONSTRAINTS_CONSTRAINT_ROW_FRICTION_HPP
