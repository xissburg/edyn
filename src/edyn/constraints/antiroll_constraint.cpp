#include "edyn/constraints/antiroll_constraint.hpp"
#include "edyn/dynamics/row_cache.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/math/math.hpp"
#include "edyn/math/transform.hpp"

namespace edyn {

void antiroll_constraint::prepare(
    const entt::registry &, entt::entity,
    constraint_row_prep_cache &cache, scalar dt,
    const constraint_body &bodyA, const constraint_body &bodyB, const constraint_body &bodyC) {

    auto ctrl_armA = to_world_space(m_ctrl_arm_pivotA, bodyA.origin, bodyA.orn);
    auto ctrl_armB = to_world_space(m_ctrl_arm_pivotB, bodyB.origin, bodyB.orn);

    // Calculate control arm direction vector to build basis.
    auto ctrl_arm_dir = ctrl_armB - ctrl_armA;
    auto ctrl_arm_len = length(ctrl_arm_dir);
    ctrl_arm_dir /= ctrl_arm_len;

    auto rA = ctrl_armB - bodyA.pos;
    auto rB = ctrl_armB - bodyB.pos;

    // Z axis points forward.
    auto chassis_z = quaternion_z(bodyA.orn);
    // X axis points to the left.
    auto chassis_x = quaternion_x(bodyA.orn);

    // Calculate pivot point on control arm using basis.
    scalar side = m_ctrl_arm_pivotA.x > 0 ? 1 : -1;
    auto ctrl_arm_x = ctrl_arm_dir * side;
    auto ctrl_arm_y = cross(chassis_z, ctrl_arm_x);
    auto ctrl_arm_basis = matrix3x3_columns(ctrl_arm_x, ctrl_arm_y, chassis_z);
    auto ctrl_arm_pivot_rel = ctrl_arm_basis * m_pivot_ctrl_arm;
    auto ctrl_arm_pivot = ctrl_armA + ctrl_arm_pivot_rel;

    // Do the same for the control arm on the other side.
    auto other_ctrl_armA = to_world_space(m_other_ctrl_arm_pivotA, bodyA.origin, bodyA.orn);
    auto other_ctrl_armC = to_world_space(m_other_ctrl_arm_pivotC, bodyC.origin, bodyC.orn);
    auto other_ctrl_arm_dir = other_ctrl_armA - other_ctrl_armC;
    auto other_ctrl_arm_len = length(other_ctrl_arm_dir);
    other_ctrl_arm_dir /= other_ctrl_arm_len;

    auto other_ctrl_arm_x = other_ctrl_arm_dir * side;
    auto other_ctrl_arm_y = cross(chassis_z, other_ctrl_arm_x);
    auto other_ctrl_arm_basis = matrix3x3_columns(other_ctrl_arm_x, other_ctrl_arm_y, chassis_z);
    auto other_ctrl_arm_pivot_rel = other_ctrl_arm_basis * m_other_pivot_ctrl_arm;
    auto other_ctrl_arm_pivot = other_ctrl_armA + other_ctrl_arm_pivot_rel;

    auto pivotA = to_world_space(m_pivotA, bodyA.origin, bodyA.orn);
    auto leverB = project_direction(ctrl_arm_pivot - pivotA, chassis_x);
    auto leverC = project_direction(other_ctrl_arm_pivot - pivotA, chassis_x);

    auto lever_lenB = length(leverB);
    auto lever_lenC = length(leverC);

    EDYN_ASSERT(lever_lenB > EDYN_EPSILON);
    EDYN_ASSERT(lever_lenC > EDYN_EPSILON);

    leverB /= lever_lenB;
    leverC /= lever_lenC;

    // Force is generated in the direction of lever arm deflection, which
    // attempts to make the angle between them go to zero.
    // TODO: sounds like an imprecise statement. The force should be applied
    // in a direction orthogonal to the lever arm, i.e. cross(leverB, chassis_x)
    // towards the location of the opposite lever arm. Though, perhaps, this
    // simpler form is close enough. Todo: measure it.
    auto force_dir = leverC - leverB;
    auto force_dir_len_sqr = length_sqr(force_dir);

    if (force_dir_len_sqr > EDYN_EPSILON) {
        force_dir /= std::sqrt(force_dir_len_sqr);
    } else {
        force_dir = cross(leverC, chassis_x);
    }

    auto angle = to_degrees(std::acos(std::clamp(dot(leverB, leverC), scalar(-1), scalar(1))));
    auto torque = m_stiffness * angle;
    auto force = torque / lever_lenB;

    // Apply corrective impulse at the wheel pivot along the direction
    // normal to the control arm, similar to spring-damper_constraint.
    auto cos_theta = dot(ctrl_arm_y, force_dir);
    auto ctrl_arm_pivot_horizontal_dist = m_pivot_ctrl_arm.x * side;
    auto ctrl_arm_pivot_ratio = ctrl_arm_pivot_horizontal_dist / ctrl_arm_len;
    auto ctrl_arm_pivot_ratio_inv = scalar(1) / ctrl_arm_pivot_ratio;
    auto lever_term = ctrl_arm_pivot_ratio * cos_theta;
    auto antiroll_impulse = std::abs(force * lever_term) * dt;

    auto d = ctrl_arm_y;
    auto p = cross(rA, d);
    auto q = cross(rB, d);

    auto &row = cache.add_row();
    row.J = {d, p, -d, -q};
    row.lower_limit = -antiroll_impulse;
    row.upper_limit = antiroll_impulse;
    row.impulse = applied_impulse;

    auto &options = cache.get_options();
    // Make error inversely proportional to distance from control arm pivot.
    options.error = angle * cos_theta * ctrl_arm_pivot_ratio_inv / dt;
}

void antiroll_constraint::store_applied_impulses(const std::vector<scalar> &impulses) {
    applied_impulse = impulses[0];
}

}
