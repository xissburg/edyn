#ifndef EDYN_COMP_CON_ROW_ITER_DATA
#define EDYN_COMP_CON_ROW_ITER_DATA

#include "edyn/math/matrix3x3.hpp"

namespace edyn {

struct delta_linvel;
struct delta_angvel;

struct con_row_iter_data {
    // Jacobian diagonals.
    std::array<vector3, 2 * 2> J;

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

    // Inverse masses and inertias used during the solver iterations. Values
    // do not necessarily represent the latest state. Query values for the
    // rigid body from the registry instead.
    scalar inv_mA, inv_mB;
    matrix3x3 inv_IA, inv_IB;

    // Reference to delta velocities used during solver iterations. It is not
    // safe to dereference these outside of the solver context.
    delta_linvel *dvA, *dvB;
    delta_angvel *dwA, *dwB;
};

}

#endif // EDYN_COMP_CON_ROW_ITER_DATA