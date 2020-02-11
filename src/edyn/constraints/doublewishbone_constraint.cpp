#include "edyn/constraints/doublewishbone_constraint.hpp"
#include "edyn/comp/constraint.hpp"
#include "edyn/comp/constraint_row.hpp"
#include "edyn/comp/relation.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/math/math.hpp"
#include <entt/entt.hpp>

namespace edyn {

void doublewishbone_constraint::init(entt::entity, constraint &con, const relation &rel, entt::registry &registry) {
    con.num_rows = steerable ? 5 : 6;

    for (size_t i = 0; i < con.num_rows; ++i) {
        con.row[i] = registry.create();
        auto &row = registry.assign<constraint_row>(con.row[i]);
        row.entity = rel.entity;
        row.priority = 100;
    }
}

void doublewishbone_constraint::prepare(entt::entity, constraint &con, const relation &rel, entt::registry &registry, scalar dt) {
    EDYN_ASSERT(side == 1 || side == -1);

    auto &pA = registry.get<const position>(rel.entity[0]);
    auto &qA = registry.get<const orientation>(rel.entity[0]);
    auto &pB = registry.get<const position>(rel.entity[1]);
    auto &qB = registry.get<const orientation>(rel.entity[1]);
    
    // Upper control arm locations.
    auto urA = rotate(qA, upper_pivotA);
    auto uposA = pA + urA;

    auto urB = rotate(qB, upper_pivotB);
    auto uposB = pB + urB;

    auto ud = uposA - uposB;
    auto ul2 = length2(ud);

    // Lower control arm locations.
    auto lrA = rotate(qA, lower_pivotA);
    auto lposA = pA + lrA;

    auto lrB = rotate(qB, lower_pivotB);
    auto lposB = pB + lrB;

    auto ld = lposA - lposB;
    auto ll2 = length2(ld);

    // Z axis points forward.
    auto chassis_z = rotate(qA, vector3_z);

    // Wheel rotation axis.
    auto wheel_x = rotate(qB, vector3_x * side);

    size_t row_idx = 0;

    // Upper control arm distance constraint.
    {
        auto &row = registry.get<constraint_row>(con.row[row_idx++]);
        row.J = {ud, cross(urA, ud), -ud, -cross(urB, ud)};
        row.error = 0.5 * (ul2 - upper_length * upper_length) / dt;
        row.lower_limit = -large_scalar;
        row.upper_limit = large_scalar;
    }

    // Lower control arm distance constraint
    {
        auto &row = registry.get<constraint_row>(con.row[row_idx++]);
        row.J = {ld, cross(lrA, ld), -ld, -cross(lrB, ld)};
        row.error = 0.5 * (ll2 - lower_length * lower_length) / dt;
        row.lower_limit = -large_scalar;
        row.upper_limit = large_scalar;
    }
    
    // Constrain upper pivot on wheel to a plane that passes through upper pivot
    // on chassis with normal equals chassis' z axis
    {
        auto p = cross(urA, chassis_z) + cross(chassis_z, ud);
        auto q = cross(urB, chassis_z);

        auto &row = registry.get<constraint_row>(con.row[row_idx++]);
        row.J = {chassis_z, p, -chassis_z, -q};
        row.error = dot(ud, chassis_z) / dt;
        row.lower_limit = -large_scalar;
        row.upper_limit = large_scalar;
    }

    // Constrain lower pivot on wheel to a plane that passes through lower pivot
    // on chassis with normal equals chassis' z axis
    {
        auto p = cross(lrA, chassis_z) + cross(chassis_z, ld);
        auto q = cross(lrB, chassis_z);

        auto &row = registry.get<constraint_row>(con.row[row_idx++]);
        row.J = {chassis_z, p, -chassis_z, -q};
        row.error = dot(ld, chassis_z) / dt;
        
        row.lower_limit = -large_scalar;
        row.upper_limit = large_scalar;
    }
    
    auto mrA = (urA + lrA) / 2;
    auto mrB = (urB + lrB) / 2;
    auto mposA = (uposA + lposA) / 2;
    auto mposB = (uposB + lposB) / 2;
    auto md = mposA - mposB;

    // Constrain the middle of the axis on the wheel to always stay in front of 
    // a plane passing through the middle of the axis on the chassis with normal
    // pointing outside the vehicle.
    {
        auto chassis_x = rotate(qA, vector3_x * side);
        auto p = cross(mrA, chassis_x) + cross(chassis_x, md);
        auto q = cross(mrB, chassis_x);

        auto &row = registry.get<constraint_row>(con.row[row_idx++]);
        row.J = {chassis_x, p, -chassis_x, -q};
        row.error = 0.2 * (dot(md, chassis_x) + 0.2) / dt; // be gentle
        row.lower_limit = -large_scalar;
        row.upper_limit = 0;
    }

    if (!steerable) {
        // Constrain wheel rotation axis to a plane that passes through upper pivot
        // on chassis with normal equals chassis' z axis
        auto q = cross(chassis_z, wheel_x);

        auto &row = registry.get<constraint_row>(con.row[row_idx++]);
        row.J = {vector3_zero, q, vector3_zero, -q};
        row.error = dot(chassis_z, wheel_x) / dt;
        row.lower_limit = -large_scalar;
        row.upper_limit = large_scalar;
    }
}

}