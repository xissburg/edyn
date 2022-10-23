#include "edyn/constraints/doublewishbone_constraint.hpp"
#include "edyn/comp/origin.hpp"
#include "edyn/constraints/constraint_row.hpp"
#include "edyn/dynamics/row_cache.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/delta_linvel.hpp"
#include "edyn/comp/delta_angvel.hpp"
#include "edyn/comp/center_of_mass.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/math/math.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/math/transform.hpp"
#include "edyn/util/constraint_util.hpp"
#include <entt/entt.hpp>

namespace edyn {

void doublewishbone_constraint::prepare(
    const entt::registry &, entt::entity,
    constraint_row_prep_cache &cache, scalar dt,
    const constraint_body &bodyA, const constraint_body &bodyB) {

    // Upper control arm locations.
    auto upivotA = to_world_space(upper_pivotA, bodyA.origin, bodyA.orn);
    auto urA = upivotA - bodyA.pos;

    auto upivotB = to_world_space(upper_pivotB, bodyB.origin, bodyB.orn);
    auto urB = upivotB - bodyB.pos;

    auto ud = upivotA - upivotB;
    auto ul2 = length_sqr(ud);

    // Lower control arm locations.
    auto lpivotA = to_world_space(lower_pivotA, bodyA.origin, bodyA.orn);
    auto lrA = lpivotA - bodyA.pos;

    auto lpivotB = to_world_space(lower_pivotB, bodyB.origin, bodyB.orn);
    auto lrB = lpivotB - bodyB.pos;

    auto ld = lpivotA - lpivotB;
    auto ll2 = length_sqr(ld);

    // Z axis points forward.
    auto chassis_z = rotate(bodyA.orn, vector3_z);

    // Wheel rotation axis.
    scalar side = lower_pivotA.x > 0 ? 1 : -1;
    auto wheel_x = rotate(bodyB.orn, vector3_x * side);

    // Upper control arm distance constraint.
    {
        auto &row = cache.add_row();
        row.J = {ud, cross(urA, ud), -ud, -cross(urB, ud)};
        row.lower_limit = -large_scalar;
        row.upper_limit = large_scalar;
        row.impulse = applied_impulse.upper_ctrl_arm_distance;

        cache.get_options().error = 0.5 * (ul2 - upper_length * upper_length) / dt;
    }

    // Lower control arm distance constraint
    {
        auto &row = cache.add_row();
        row.J = {ld, cross(lrA, ld), -ld, -cross(lrB, ld)};
        row.lower_limit = -large_scalar;
        row.upper_limit = large_scalar;
        row.impulse = applied_impulse.lower_ctrl_arm_distance;

        cache.get_options().error = 0.5 * (ll2 - lower_length * lower_length) / dt;
    }

    // Constrain upper pivot on wheel to a plane that passes through upper pivot
    // on chassis with normal equals chassis' z axis
    {
        auto p = cross(urA, chassis_z) + cross(chassis_z, ud);
        auto q = cross(urB, chassis_z);

        auto &row = cache.add_row();
        row.J = {chassis_z, p, -chassis_z, -q};
        row.lower_limit = -large_scalar;
        row.upper_limit = large_scalar;
        row.impulse = applied_impulse.upper_ctrl_arm_plane;

        cache.get_options().error = dot(ud, chassis_z) / dt;
    }

    // Constrain lower pivot on wheel to a plane that passes through lower pivot
    // on chassis with normal equals chassis' z axis
    {
        auto p = cross(lrA, chassis_z) + cross(chassis_z, ld);
        auto q = cross(lrB, chassis_z);

        auto &row = cache.add_row();
        row.J = {chassis_z, p, -chassis_z, -q};
        row.lower_limit = -large_scalar;
        row.upper_limit = large_scalar;
        row.impulse = applied_impulse.lower_ctrl_arm_plane;

        cache.get_options().error = dot(ld, chassis_z) / dt;
    }

    auto mrA = (urA + lrA) / 2;
    auto mrB = (urB + lrB) / 2;
    auto mposA = (upivotA + lpivotA) / 2;
    auto mposB = (upivotB + lpivotB) / 2;
    auto md = mposA - mposB;

    // Constrain the middle of the axis on the wheel to always stay in front of
    // a plane passing through the middle of the axis on the chassis with normal
    // pointing outside the vehicle.
    {
        auto chassis_x = rotate(bodyA.orn, vector3_x * side);
        auto p = cross(mrA, chassis_x) + cross(chassis_x, md);
        auto q = cross(mrB, chassis_x);

        auto &row = cache.add_row();
        row.J = {chassis_x, p, -chassis_x, -q};
        row.lower_limit = -large_scalar;
        row.upper_limit = 0;
        row.impulse = applied_impulse.keep_outside;

        cache.get_options().error = 0.2 * (dot(md, chassis_x) + 0.2) / dt; // be gentle
    }

    if (!steerable) {
        // Constrain wheel rotation axis to a plane that passes through upper pivot
        // on chassis with normal equals chassis' z axis
        auto q = cross(chassis_z, wheel_x);

        auto &row = cache.add_row();
        row.J = {vector3_zero, q, vector3_zero, -q};
        row.lower_limit = -large_scalar;
        row.upper_limit = large_scalar;
        row.impulse = applied_impulse.anti_steering;

        cache.get_options().error = dot(chassis_z, wheel_x) / dt;
    }
}

void doublewishbone_constraint::store_applied_impulses(const std::vector<scalar> &impulses) {
    int row_idx = 0;
    applied_impulse.upper_ctrl_arm_distance = impulses[row_idx++];
    applied_impulse.lower_ctrl_arm_distance = impulses[row_idx++];
    applied_impulse.upper_ctrl_arm_plane = impulses[row_idx++];
    applied_impulse.lower_ctrl_arm_plane = impulses[row_idx++];
    applied_impulse.keep_outside = impulses[row_idx++];

    if (!steerable) {
        applied_impulse.anti_steering = impulses[row_idx++];
    }
}

}
