#include "edyn/constraints/point_constraint.hpp"
#include "edyn/math/transform.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/dynamics/row_cache.hpp"

namespace edyn {

void point_constraint::prepare(
    const entt::registry &, entt::entity,
    constraint_row_prep_cache &cache, scalar dt,
    const constraint_body &bodyA, const constraint_body &bodyB) {

    auto pivotA = to_world_space(pivot[0], bodyA.origin, bodyA.orn);
    auto pivotB = to_world_space(pivot[1], bodyB.origin, bodyB.orn);
    auto rA = pivotA - bodyA.pos;
    auto rB = pivotB - bodyB.pos;

    auto rA_skew = skew_matrix(rA);
    auto rB_skew = skew_matrix(rB);
    constexpr auto I = matrix3x3_identity;
    auto num_rows = size_t{3};

    for (size_t i = 0; i < num_rows; ++i) {
        auto &row = cache.add_row();
        row.J = {I.row[i], -rA_skew.row[i], -I.row[i], rB_skew.row[i]};
        row.lower_limit = -EDYN_SCALAR_MAX;
        row.upper_limit = EDYN_SCALAR_MAX;
        row.impulse = impulse[i];

        auto &options = cache.get_options();
        options.error = (pivotA[i] - pivotB[i]) / dt;
    }

    if (friction_torque > 0) {
        auto spin_axis = bodyA.angvel - bodyB.angvel;

        if (try_normalize(spin_axis)) {
            auto &row = cache.add_row();
            row.J = {vector3_zero, spin_axis, vector3_zero, -spin_axis};
            row.impulse = impulse[++num_rows];

            auto friction_impulse = friction_torque * dt;
            row.lower_limit = -friction_impulse;
            row.upper_limit = friction_impulse;
        }
    }
}

}
