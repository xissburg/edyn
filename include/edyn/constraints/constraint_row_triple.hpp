#ifndef EDYN_CONSTRAINTS_CONSTRAINT_ROW_TRIPLE_HPP
#define EDYN_CONSTRAINTS_CONSTRAINT_ROW_TRIPLE_HPP

#include <array>
#include "edyn/comp/spin.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/config/constants.hpp"
#include "edyn/comp/delta_linvel.hpp"
#include "edyn/comp/delta_angvel.hpp"

namespace edyn {

struct constraint_row_options;

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

void prepare_row(constraint_row_triple &row,
                 const constraint_row_options &options,
                 const vector3 &linvelA, const vector3 &angvelA, scalar spinA,
                 const vector3 &linvelB, const vector3 &angvelB, scalar spinB,
                 const vector3 &linvelC, const vector3 &angvelC, scalar spinC);

void apply_row_impulse(scalar impulse, constraint_row_triple &row);

void warm_start(constraint_row_triple &row);

scalar solve(constraint_row_triple &row);

}

#endif // EDYN_CONSTRAINTS_CONSTRAINT_ROW_TRIPLE_HPP
