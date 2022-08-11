#include "edyn/constraints/constraint_row_friction.hpp"
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

}
