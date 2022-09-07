#ifndef EDYN_CONSTRAINTS_CONSTRAINT_ROW_TRIPLE_HPP
#define EDYN_CONSTRAINTS_CONSTRAINT_ROW_TRIPLE_HPP

#include <array>
#include "edyn/math/vector3.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/config/constants.hpp"
#include "edyn/comp/delta_linvel.hpp"
#include "edyn/comp/delta_angvel.hpp"
#include "edyn/comp/delta_spin.hpp"

namespace edyn {

struct constraint_row_triple {
    // Jacobian diagonals.
    std::array<vector3, 6> J;

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
};

inline scalar solve3(constraint_row_triple &row) {
    auto dsA = vector3_zero;
    auto dsB = vector3_zero;
    auto dsC = vector3_zero;

    if (row.use_spin[0] && row.dsA != nullptr) {
        dsA = row.spin_axis[0] * *row.dsA;
    }

    if (row.use_spin[1] && row.dsB != nullptr) {
        dsB = row.spin_axis[1] * *row.dsB;
    }

    if (row.use_spin[2] && row.dsC != nullptr) {
        dsC = row.spin_axis[2] * *row.dsC;
    }

    auto delta_relvel = dot(row.J[0], *row.dvA) +
                        dot(row.J[1], *row.dwA + dsA) +
                        dot(row.J[2], *row.dvB) +
                        dot(row.J[3], *row.dwB + dsB) +
                        dot(row.J[4], *row.dvC) +
                        dot(row.J[5], *row.dwC + dsC);
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

#endif // EDYN_CONSTRAINTS_CONSTRAINT_ROW_TRIPLE_HPP
