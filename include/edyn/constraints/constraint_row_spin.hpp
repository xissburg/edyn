#ifndef EDYN_CONSTRAINTS_CONSTRAINT_ROW_SPIN_HPP
#define EDYN_CONSTRAINTS_CONSTRAINT_ROW_SPIN_HPP

#include <array>
#include "edyn/math/vector3.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/config/constants.hpp"
#include "edyn/comp/delta_linvel.hpp"
#include "edyn/comp/delta_angvel.hpp"

namespace edyn {

struct constraint_row_spin {
    // Jacobian diagonals.
    std::array<vector3, 2> J;

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

}

#endif // EDYN_CONSTRAINTS_CONSTRAINT_ROW_SPIN_HPP
