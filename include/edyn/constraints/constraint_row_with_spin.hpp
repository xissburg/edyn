#ifndef EDYN_CONSTRAINTS_CONSTRAINT_ROW_WITH_SPIN_HPP
#define EDYN_CONSTRAINTS_CONSTRAINT_ROW_WITH_SPIN_HPP

#include <array>
#include "edyn/comp/spin.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/config/constants.hpp"
#include "edyn/comp/delta_linvel.hpp"
#include "edyn/comp/delta_angvel.hpp"

namespace edyn {

struct constraint_row_options;

struct constraint_row_with_spin {
    // Jacobian diagonals.
    std::array<vector3, 4> J;

    // Effective mass (J M^-1 J^T)^-1.
    scalar eff_mass;

    // Right hand side Jv + bias.
    scalar rhs;

    // Lower and upper limit of impulses to be applied while solving constraints.
    scalar lower_limit;
    scalar upper_limit;

    // Last applied impulse. Used for warm-starting and can be used to measure
    // strength of impulse applied.
    scalar impulse;

    // Inverse masses and inertias used during the solver iterations.
    scalar inv_mA, inv_mB;
    matrix3x3 inv_IA, inv_IB;

    // Reference to delta velocities used during solver iterations. It is not
    // safe to dereference these outside of the solver update context.
    delta_linvel *dvA, *dvB;
    delta_angvel *dwA, *dwB;
    delta_spin *dsA, *dsB;

    std::array<bool, 2> use_spin {false, false};
    vector3 spin_axis[2];
};

void prepare_row(constraint_row_with_spin &row,
                 const constraint_row_options &options,
                 const vector3 &linvelA, const vector3 &angvelA, scalar spinA,
                 const vector3 &linvelB, const vector3 &angvelB, scalar spinB);

void apply_row_impulse(scalar impulse, constraint_row_with_spin &row);

void warm_start(constraint_row_with_spin &row);

scalar solve(constraint_row_with_spin &row);

}

#endif // EDYN_CONSTRAINTS_CONSTRAINT_ROW_WITH_SPIN_HPP
