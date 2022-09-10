#include "edyn/constraints/constraint_row.hpp"
#include "edyn/constraints/constraint_row_options.hpp"

namespace edyn {

void prepare_row(constraint_row &row,
                 const constraint_row_options &options,
                 const vector3 &linvelA, const vector3 &angvelA,
                 const vector3 &linvelB, const vector3 &angvelB) {
    auto J_invM_JT = dot(row.J[0], row.J[0]) * row.inv_mA +
                     dot(row.inv_IA * row.J[1], row.J[1]) +
                     dot(row.J[2], row.J[2]) * row.inv_mB +
                     dot(row.inv_IB * row.J[3], row.J[3]);
    row.eff_mass = 1 / J_invM_JT;

    auto relvel = dot(row.J[0], linvelA) +
                  dot(row.J[1], angvelA) +
                  dot(row.J[2], linvelB) +
                  dot(row.J[3], angvelB);

    row.rhs = -(options.error * options.erp + relvel * (1 + options.restitution));
}

void apply_row_impulse(scalar impulse, constraint_row &row) {
    // Apply linear impulse.
    *row.dvA += row.inv_mA * row.J[0] * impulse;
    *row.dvB += row.inv_mB * row.J[2] * impulse;

    // Apply angular impulse.
    *row.dwA += row.inv_IA * row.J[1] * impulse;
    *row.dwB += row.inv_IB * row.J[3] * impulse;
}

void warm_start(constraint_row &row) {
    apply_row_impulse(row.impulse, row);
}

scalar solve(constraint_row &row) {
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
