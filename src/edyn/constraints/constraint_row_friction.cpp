#include "edyn/constraints/constraint_row_friction.hpp"
#include "edyn/constraints/constraint_row_with_spin.hpp"
#include "edyn/dynamics/row_cache.hpp"
#include "edyn/math/math.hpp"
#include "edyn/math/vector2.hpp"
#include "edyn/util/constraint_util.hpp"
#include "edyn/comp/delta_linvel.hpp"
#include "edyn/comp/delta_angvel.hpp"

namespace edyn {

void solve_friction(constraint_row_friction &friction_row, const std::vector<constraint_row> &row_cache) {
    // Impulse is limited by the length of a 2D vector to assure a friction circle.
    vector2 delta_impulse;
    vector2 impulse;
    auto &normal_row = row_cache[friction_row.normal_row_index];

    for (auto i = 0; i < 2; ++i) {
        auto &row_i = friction_row.row[i];
        auto delta_relspd = get_relative_speed(row_i.J,
                                               *normal_row.dvA, *normal_row.dwA,
                                               *normal_row.dvB, *normal_row.dwB);
        delta_impulse[i] = (row_i.rhs - delta_relspd) * row_i.eff_mass;
        impulse[i] = row_i.impulse + delta_impulse[i];
    }

    auto impulse_len_sqr = length_sqr(impulse);
    auto max_impulse_len = friction_row.friction_coefficient * normal_row.impulse;

    // Limit impulse by normal load.
    if (impulse_len_sqr > square(max_impulse_len)) {
        auto impulse_len = std::sqrt(impulse_len_sqr);

        if (impulse_len > EDYN_EPSILON) {
            impulse = impulse / impulse_len * max_impulse_len;
        } else {
            impulse = {0, 0};
        }

        for (auto i = 0; i < 2; ++i) {
            delta_impulse[i] = impulse[i] - friction_row.row[i].impulse;
        }
    }

    // Apply delta impulse.
    for (auto i = 0; i < 2; ++i) {
        auto &row_i = friction_row.row[i];
        row_i.impulse = impulse[i];

        *normal_row.dvA += normal_row.inv_mA * row_i.J[0] * delta_impulse[i];
        *normal_row.dwA += normal_row.inv_IA * row_i.J[1] * delta_impulse[i];
        *normal_row.dvB += normal_row.inv_mB * row_i.J[2] * delta_impulse[i];
        *normal_row.dwB += normal_row.inv_IB * row_i.J[3] * delta_impulse[i];
    }
}

void warm_start(constraint_row_friction &friction_row, const std::vector<constraint_row> &row_cache) {
    auto &normal_row = row_cache[friction_row.normal_row_index];

    for (int i = 0; i < 2; ++i) {
        auto &row_i = friction_row.row[i];
        *normal_row.dvA += normal_row.inv_mA * row_i.J[0] * row_i.impulse;
        *normal_row.dwA += normal_row.inv_IA * row_i.J[1] * row_i.impulse;
        *normal_row.dvB += normal_row.inv_mB * row_i.J[2] * row_i.impulse;
        *normal_row.dwB += normal_row.inv_IB * row_i.J[3] * row_i.impulse;
    }
}

void apply_friction_angular_impulse(scalar impulse,
                                    const constraint_row_friction::individual_row &row,
                                    const constraint_row_with_spin &normal_row,
                                    size_t ent_idx) {
    auto idx_J = ent_idx * 2 + 1;
    matrix3x3 inv_I;
    delta_angvel *dw;
    delta_spin *ds;

    if (ent_idx == 0) {
        inv_I = normal_row.inv_IA;
        dw = normal_row.dwA;
        ds = normal_row.dsA;
    } else {
        inv_I = normal_row.inv_IB;
        dw = normal_row.dwB;
        ds = normal_row.dsB;
    }

    auto delta = inv_I * row.J[idx_J] * impulse;

    if (normal_row.use_spin[ent_idx] && ds) {
        // Split delta in a spin component and an angular component.
        auto spin = dot(normal_row.spin_axis[ent_idx], delta);
        *ds += spin;
        // Subtract spin to obtain only angular component.
        *dw += delta - normal_row.spin_axis[ent_idx] * spin;
    } else {
        *dw += delta;
    }
}

void apply_friction_impulse(scalar impulse, const constraint_row_friction::individual_row &row, const constraint_row_with_spin &normal_row) {
    // Apply linear impulse.
    *normal_row.dvA += normal_row.inv_mA * row.J[0] * impulse;
    *normal_row.dvB += normal_row.inv_mB * row.J[2] * impulse;

    // Apply angular impulse.
    apply_friction_angular_impulse(impulse, row, normal_row, 0);
    apply_friction_angular_impulse(impulse, row, normal_row, 1);
}

void solve_friction(constraint_row_friction &friction_row, const std::vector<constraint_row_with_spin> &row_cache) {
    // Impulse is limited by the length of a 2D vector to assure a friction circle.
    vector2 delta_impulse;
    vector2 impulse;
    auto &normal_row = row_cache[friction_row.normal_row_index];

    auto dsA = vector3_zero;
    auto dsB = vector3_zero;

    if (normal_row.use_spin[0] && normal_row.dsA != nullptr) {
        dsA = normal_row.spin_axis[0] * *normal_row.dsA;
    }

    if (normal_row.use_spin[1] && normal_row.dsB != nullptr) {
        dsB = normal_row.spin_axis[1] * *normal_row.dsB;
    }

    for (auto i = 0; i < 2; ++i) {
        auto &row_i = friction_row.row[i];
        auto delta_relspd = get_relative_speed(row_i.J,
                                               *normal_row.dvA, *normal_row.dwA + dsA,
                                               *normal_row.dvB, *normal_row.dwB + dsB);
        delta_impulse[i] = (row_i.rhs - delta_relspd) * row_i.eff_mass;
        impulse[i] = row_i.impulse + delta_impulse[i];
    }

    auto impulse_len_sqr = length_sqr(impulse);
    auto max_impulse_len = friction_row.friction_coefficient * normal_row.impulse;

    // Limit impulse by normal load.
    if (impulse_len_sqr > square(max_impulse_len)) {
        auto impulse_len = std::sqrt(impulse_len_sqr);

        if (impulse_len > EDYN_EPSILON) {
            impulse = impulse / impulse_len * max_impulse_len;
        } else {
            impulse = {0, 0};
        }

        for (auto i = 0; i < 2; ++i) {
            delta_impulse[i] = impulse[i] - friction_row.row[i].impulse;
        }
    }

    // Apply delta impulse.
    for (auto i = 0; i < 2; ++i) {
        auto &row_i = friction_row.row[i];
        row_i.impulse = impulse[i];
        apply_friction_impulse(delta_impulse[i], row_i, normal_row);
    }
}

void warm_start(constraint_row_friction &friction_row, constraint_row_with_spin &normal_row) {
    for (auto i = 0; i < 2; ++i) {
        auto &row_i = friction_row.row[i];
        apply_friction_impulse(row_i.impulse, row_i, normal_row);
    }
}

}
