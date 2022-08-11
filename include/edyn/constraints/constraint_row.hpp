#ifndef EDYN_COMP_CONSTRAINT_ROW_HPP
#define EDYN_COMP_CONSTRAINT_ROW_HPP

#include <array>
#include "edyn/math/vector3.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/config/constants.hpp"
#include "edyn/comp/delta_linvel.hpp"
#include "edyn/comp/delta_angvel.hpp"

namespace edyn {

/**
 * `constraint_row` contains all and only the information that's required
 * during the constraint solver iterations for better cache use and to avoid
 * waste.
 */
struct constraint_row {
    // Jacobian diagonals.
    std::array<vector3, 2 * max_constrained_entities> J;

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

inline scalar solve(constraint_row &row) {
    auto delta_relvel = dot(row.J[0], *row.dvA) +
                        dot(row.J[1], *row.dwA) +
                        dot(row.J[2], *row.dvB) +
                        dot(row.J[3], *row.dwB);
    auto delta_impulse = (row.rhs - delta_relvel) * row.eff_mass;
    auto impulse = row.impulse + delta_impulse;

    if (impulse < row.lower_limit) {
        delta_impulse = row.lower_limit - row.impulse;
        row.impulse = row.lower_limit;
    } else if (impulse > row.upper_limit) {
        delta_impulse = row.upper_limit - row.impulse;
        row.impulse = row.upper_limit;
    } else {
        row.impulse = impulse;
    }

    return delta_impulse;
}

}

#endif // EDYN_COMP_CONSTRAINT_ROW_HPP
