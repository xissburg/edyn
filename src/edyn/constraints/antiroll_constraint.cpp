#include "edyn/constraints/antiroll_constraint.hpp"
#include "edyn/comp/constraint.hpp"
#include "edyn/comp/constraint_row.hpp"
#include "edyn/comp/relation.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/math/math.hpp"
#include <entt/entt.hpp>

namespace edyn {

void antiroll_constraint::init(entt::entity, constraint &con, const relation &rel, entt::registry &registry) {
    con.num_rows = 1;
    con.row[0] = registry.create();
    auto &row = registry.assign<constraint_row>(con.row[0]);
    row.entity = rel.entity;
    row.priority = 500;

    EDYN_ASSERT(third_entity != entt::null);
}

void antiroll_constraint::prepare(entt::entity, constraint &con, const relation &rel, entt::registry &registry, scalar dt) {
    auto &pA = registry.get<const position>(rel.entity[0]);
    auto &qA = registry.get<const orientation>(rel.entity[0]);
    auto rA = rotate(qA, pivotA);
    auto posA = pA + rA;

    auto &pB = registry.get<const position>(rel.entity[1]);
    auto &qB = registry.get<const orientation>(rel.entity[1]);
    auto rB = rotate(qB, ctrl_arm_pivotB);
    auto posB = pB + rB;

    auto &pC = registry.get<const position>(third_entity);
    auto &qC = registry.get<const orientation>(third_entity);
    auto rC = rotate(qC, other_ctrl_arm_pivotC);
    auto posC = pC + rC;

    // Z axis points forward.
    auto chassis_z = rotate(qA, vector3_z);

    // Calculate control arm direction vector to build basis.
    auto ctrl_armA = pA + rotate(qA, ctrl_arm_pivotA);
    auto ctrl_armB = posB;
    auto ctrl_arm_dir = ctrl_armB - ctrl_armA;
    auto ctrl_arm_len = length(ctrl_arm_dir);
    ctrl_arm_dir /= ctrl_arm_len;

    // Calculate pivot point on control arm using basis.
    auto ctrl_arm_x = ctrl_arm_dir * side;
    auto ctrl_arm_y = cross(chassis_z, ctrl_arm_x);
    auto ctrl_arm_basis = matrix3x3_columns(ctrl_arm_x, ctrl_arm_y, chassis_z);
    auto ctrl_arm_pivot_rel = ctrl_arm_basis * ctrl_arm_pivot;
    auto ctrl_arm_pivot = ctrl_armA + ctrl_arm_pivot_rel;

    // Do the same for the control arm on the other side.
    auto other_ctrl_armA = pA + rotate(qA, other_ctrl_arm_pivotA);
    auto other_ctrl_armC = posC;
    auto other_ctrl_arm_dir = other_ctrl_armA - other_ctrl_armC;
    auto other_ctrl_arm_len = length(other_ctrl_arm_dir);
    other_ctrl_arm_dir /= other_ctrl_arm_len;

    auto other_ctrl_arm_x = other_ctrl_arm_dir * side;
    auto other_ctrl_arm_y = cross(chassis_z, other_ctrl_arm_x);
    auto other_ctrl_arm_basis = matrix3x3_columns(other_ctrl_arm_x, other_ctrl_arm_y, chassis_z);
    auto other_ctrl_arm_pivot_rel = other_ctrl_arm_basis * other_ctrl_arm_pivot;
    auto other_ctrl_arm_pivot = other_ctrl_armA + other_ctrl_arm_pivot_rel;

    auto dB = ctrl_arm_pivot - posA;
    auto dC = other_ctrl_arm_pivot - posA;
    auto chassis_x = rotate(qA, vector3_x);
    auto d_projB = dB - chassis_x * dot(dB, chassis_x);
    auto d_projC = dC - chassis_x * dot(dC, chassis_x);
    
    auto lever = std::max(length(d_projB), EDYN_EPSILON);
    d_projB /= lever;
    
    d_projC = normalize(d_projC);
    
    // Apply impulses in the direction of deformation.
    auto n = d_projC - d_projB;
    
    if (length2(n) <= EDYN_EPSILON) {
        n = cross(d_projC, chassis_x);
    }
    
    n = normalize(n);

    auto p = cross(rA, n);
    auto q = cross(rB, n);
    auto d = std::clamp(dot(d_projB, d_projC), scalar(-1), scalar(1));
    auto angle = to_degrees(std::acos(d));
    auto impulse = std::abs(stiffness * angle / lever) * dt;

    auto &row = registry.get<constraint_row>(con.row[0]);
    row.J = {n, p, -n, -q};
    row.error = angle / dt;
    row.lower_limit = -impulse;
    row.upper_limit = impulse;
}

}