#include "edyn/constraints/constraint_row_spin_friction.hpp"

namespace edyn {

void solve_spin_friction(constraint_row_spin_friction &row, const std::vector<constraint_row> &row_cache) {
    auto &normal_row = row_cache[row.normal_row_index];
    auto max_impulse_len = row.friction_coefficient * normal_row.impulse;

    auto delta_relvel = dot(row.J[0], *normal_row.dwA) + dot(row.J[1], *normal_row.dwB);
    auto delta_impulse = (row.rhs - delta_relvel) * row.eff_mass;
    auto impulse = row.impulse + delta_impulse;
    auto lower_limit = -max_impulse_len;
    auto upper_limit = max_impulse_len;

    if (impulse < lower_limit) {
        delta_impulse = lower_limit - row.impulse;
        row.impulse = lower_limit;
    } else if (impulse > upper_limit) {
        delta_impulse = upper_limit - row.impulse;
        row.impulse = upper_limit;
    } else {
        row.impulse = impulse;
    }

    // Apply angular impulse.
    *normal_row.dwA += normal_row.inv_IA * row.J[0] * delta_impulse;
    *normal_row.dwB += normal_row.inv_IB * row.J[1] * delta_impulse;
}

void warm_start(constraint_row_spin_friction &row, const std::vector<constraint_row> &row_cache) {
    auto &normal_row = row_cache[row.normal_row_index];
    // Apply angular impulse.
    *normal_row.dwA += normal_row.inv_IA * row.J[0] * row.impulse;
    *normal_row.dwB += normal_row.inv_IB * row.J[1] * row.impulse;
}

}
