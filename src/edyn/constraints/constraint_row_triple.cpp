#include "edyn/constraints/constraint_row_triple.hpp"
#include "edyn/constraints/constraint_row_options.hpp"
#include "edyn/util/constraint_util.hpp"

namespace edyn {

void prepare_row(constraint_row_triple &row,
                 const constraint_row_options &options,
                 const vector3 &linvelA, const vector3 &angvelA, scalar spinA,
                 const vector3 &linvelB, const vector3 &angvelB, scalar spinB,
                 const vector3 &linvelC, const vector3 &angvelC, scalar spinC) {
    row.eff_mass = get_effective_mass(row.J,
                                      row.inv_mA, row.inv_IA,
                                      row.inv_mB, row.inv_IB,
                                      row.inv_mC, row.inv_IC);

    auto relvel = dot(row.J[0], linvelA) +
                  dot(row.J[1], angvelA + row.spin_axis[0] * spinA) +
                  dot(row.J[2], linvelB) +
                  dot(row.J[3], angvelB + row.spin_axis[1] * spinB) +
                  dot(row.J[4], linvelC) +
                  dot(row.J[5], angvelC + row.spin_axis[2] * spinC);

    row.rhs = -(options.error * options.erp + relvel * (1 + options.restitution));
}

static
void apply_angular_impulse(scalar impulse,
                           constraint_row_triple &row,
                           size_t ent_idx) {
    auto idx_J = ent_idx * 2 + 1;
    matrix3x3 inv_I;
    delta_angvel *dw;
    delta_spin *ds;

    if (ent_idx == 0) {
        inv_I = row.inv_IA;
        dw = row.dwA;
        ds = row.dsA;
    } else if (ent_idx == 1) {
        inv_I = row.inv_IB;
        dw = row.dwB;
        ds = row.dsB;
    } else {
        inv_I = row.inv_IC;
        dw = row.dwC;
        ds = row.dsC;
    }

    auto delta = inv_I * row.J[idx_J] * impulse;

    if (row.use_spin[ent_idx] && ds) {
        // Split delta in a spin component and an angular component.
        auto spin = dot(row.spin_axis[ent_idx], delta);
        *ds += spin;
        // Subtract spin to obtain only angular component.
        *dw += delta - row.spin_axis[ent_idx] * spin;
    } else {
        *dw += delta;
    }
}

void apply_row_impulse(scalar impulse, constraint_row_triple &row) {
    // Apply linear impulse.
    *row.dvA += row.inv_mA * row.J[0] * impulse;
    *row.dvB += row.inv_mB * row.J[2] * impulse;
    *row.dvC += row.inv_mC * row.J[4] * impulse;

    // Apply angular impulse.
    apply_angular_impulse(impulse, row, 0);
    apply_angular_impulse(impulse, row, 1);
    apply_angular_impulse(impulse, row, 2);
}

void warm_start(constraint_row_triple &row) {
    apply_row_impulse(row.impulse, row);
}

scalar solve(constraint_row_triple &row) {
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
