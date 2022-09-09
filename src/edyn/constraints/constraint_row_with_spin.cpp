#include "edyn/constraints/constraint_row_with_spin.hpp"
#include "edyn/constraints/constraint_row_options.hpp"

namespace edyn {

void prepare_row(constraint_row_with_spin &row,
                 const constraint_row_options &options,
                 const vector3 &linvelA, const vector3 &angvelA, scalar spinA,
                 const vector3 &linvelB, const vector3 &angvelB, scalar spinB) {
    auto J_invM_JT = dot(row.J[0], row.J[0]) * row.inv_mA +
                     dot(row.inv_IA * row.J[1], row.J[1]) +
                     dot(row.J[2], row.J[2]) * row.inv_mB +
                     dot(row.inv_IB * row.J[3], row.J[3]);
    row.eff_mass = 1 / J_invM_JT;

    auto relvel = dot(row.J[0], linvelA) +
                  dot(row.J[1], angvelA + row.spin_axis[0] * spinA) +
                  dot(row.J[2], linvelB) +
                  dot(row.J[3], angvelB + row.spin_axis[1] * spinB);

    row.rhs = -(options.error * options.erp + relvel * (1 + options.restitution));
}

static
void apply_angular_impulse(scalar impulse,
                           constraint_row_with_spin &row,
                           size_t ent_idx) {
    auto idx_J = ent_idx * 2 + 1;
    matrix3x3 inv_I;
    delta_angvel *dw;
    delta_spin *ds;

    if (ent_idx == 0) {
        inv_I = row.inv_IA;
        dw = row.dwA;
        ds = row.dsA;
    } else {
        inv_I = row.inv_IB;
        dw = row.dwB;
        ds = row.dsB;
    }

    auto delta = inv_I * row.J[idx_J] * impulse;

    if (ds) {
        // Split delta in a spin component and an angular component.
        auto spin = dot(row.spin_axis[ent_idx], delta);
        *ds += spin;
        // Subtract spin to obtain only angular component.
        *dw += delta - row.spin_axis[ent_idx] * spin;
    } else {
        *dw += delta;
    }
}

void apply_impulse(scalar impulse, constraint_row_with_spin &row) {
    // Apply linear impulse.
    *row.dvA += row.inv_mA * row.J[0] * impulse;
    *row.dvB += row.inv_mB * row.J[2] * impulse;

    // Apply angular impulse.
    apply_angular_impulse(impulse, row, 0);
    apply_angular_impulse(impulse, row, 1);
}

void warm_start(constraint_row_with_spin &row) {
    apply_impulse(row.impulse, row);
}

scalar solve(constraint_row_with_spin &row) {
    auto dsA = vector3_zero;
    auto dsB = vector3_zero;

    if (row.use_spin[0] && row.dsA != nullptr) {
        dsA = row.spin_axis[0] * *row.dsA;
    }

    if (row.use_spin[1] && row.dsB != nullptr) {
        dsB = row.spin_axis[1] * *row.dsB;
    }

    auto delta_relvel = dot(row.J[0], *row.dvA) +
                        dot(row.J[1], *row.dwA + dsA) +
                        dot(row.J[2], *row.dvB) +
                        dot(row.J[3], *row.dwB + dsB);
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
