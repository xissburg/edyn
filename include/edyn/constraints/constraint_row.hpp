#ifndef EDYN_COMP_CONSTRAINT_ROW_HPP
#define EDYN_COMP_CONSTRAINT_ROW_HPP

#include <array>
#include "edyn/math/vector3.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/config/constants.hpp"

namespace edyn {

struct delta_linvel;
struct delta_angvel;
struct delta_spin;

/**
 * `constraint_row` contains all and only the information that's required
 * during the constraint solver iterations for better cache use and to avoid
 * waste.
 */
struct constraint_row {
    // Jacobian diagonals.
    std::array<vector3, 2 * 3> J;

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
    scalar inv_mA, inv_mB, inv_mC;
    matrix3x3 inv_IA, inv_IB, inv_IC;

    // Reference to delta velocities used during solver iterations. It is not
    // safe to dereference these outside of the solver update context.
    delta_linvel *dvA, *dvB, *dvC;
    delta_angvel *dwA, *dwB, *dwC;
    delta_spin *dsA, *dsB, *dsC;

    std::array<bool, 3> use_spin {false, false, false};
    vector3 spin_axis[3];

    unsigned num_entities {2};
};

/**
 * Optional info to be used when setting up a constraint row.
 */
struct constraint_row_options {
    scalar error {scalar(0)};

    // Error reduction parameter.
    scalar erp {scalar(0.2)};

    scalar restitution {scalar(0)};
};

}

#endif // EDYN_COMP_CONSTRAINT_ROW_HPP
