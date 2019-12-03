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

void doublewishbone_constraint::init(constraint &con, const relation &rel, entt::registry &registry) {
    con.num_rows = steerable ? 6 : 7;

    for (size_t i = 0; i < con.num_rows; ++i) {
        auto e = registry.create();
        con.row[i] = e;
        auto &row = registry.assign<constraint_row>(e);
        row.entity = rel.entity;
    }
}

void doublewishbone_constraint::prepare(constraint &con, const relation &rel, entt::registry &registry, scalar dt) {
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
    auto udistance = std::max(length(ud), EDYN_EPSILON);
    auto udn = ud / udistance;

    // Lower control arm locations.
    auto lrA = rotate(qA, lower_pivotA);
    auto lposA = pA + lrA;

    auto lrB = rotate(qB, lower_pivotB);
    auto lposB = pB + lrB;

    auto ld = lposA - lposB;
    auto ldistance = std::max(length(ld), EDYN_EPSILON);
    auto ldn = ld / ldistance;

    // Z axis points forward.
    auto chassis_z = rotate(qA, vector3_z);

    // Wheel rotation axis.
    auto wheel_x = rotate(qB, vector3_x * side);

    size_t row_idx = 0;

    // Upper control arm distance constraint.
    {
        auto &row = registry.get<constraint_row>(con.row[row_idx++]);
        row.J = {-udn, -cross(urA, udn), udn, cross(urB, udn)};
        row.error = (udistance - upper_length) / dt;
        row.lower_limit = -large_scalar;
        row.upper_limit = large_scalar;
    }

    // Lower control arm distance constraint
    {
        auto &row = registry.get<constraint_row>(con.row[row_idx++]);
        row.J = {-ldn, -cross(lrA, ldn), ldn, cross(lrB, ldn)};
        row.error = (ldistance - lower_length) / dt;
        row.lower_limit = -large_scalar;
        row.upper_limit = large_scalar;
    }

    // Constrain upper pivot on wheel to a plane that passes through upper pivot
    // on chassis with normal equals chassis' z axis
    {
        auto p = cross(urA, chassis_z) + cross(chassis_z, udn);
        auto q = cross(urB, chassis_z);

        auto &row = registry.get<constraint_row>(con.row[row_idx++]);
        row.J = {-chassis_z, -p, chassis_z, q};
        row.error = dot(udn, chassis_z) / dt;
        row.lower_limit = -large_scalar;
        row.upper_limit = large_scalar;
    }

    // Constrain lower pivot on wheel to a plane that passes through lower pivot
    // on chassis with normal equals chassis' z axis
    {
        auto p = cross(lrA, chassis_z) + cross(chassis_z, ldn);
        auto q = cross(lrB, chassis_z);

        auto &row = registry.get<constraint_row>(con.row[row_idx++]);
        row.J = {-chassis_z, -p, chassis_z, q};
        row.error = dot(ldn, chassis_z) / dt;
        row.lower_limit = -large_scalar;
        row.upper_limit = large_scalar;
    }

    auto mrA = (urA + lrA) / 2;
    auto mrB = (urB + lrB) / 2;
    auto mposA = (uposA + lposA) / 2;
    auto mposB = (uposB + lposB) / 2;
    auto md = mposA - mposB;
    auto mdn = normalize(md);

    // Constrain the middle of the axis on the wheel to always stay in front of 
    // a plane passing through the middle of the axis on the chassis with normal
    // pointing outside the vehicle.
    {
        auto chassis_x = rotate(qA, vector3_x * side);
        auto p = cross(mrA, chassis_x) + cross(chassis_x, mdn);
        auto q = cross(mrB, chassis_x);

        auto &row = registry.get<constraint_row>(con.row[row_idx++]);
        row.J = {-chassis_x, -p, chassis_x, q};
        row.error = (dot(mdn, chassis_x) + 0.4) / dt;
        row.lower_limit = 0;
        row.upper_limit = large_scalar;
    }

    // Constrain the middle of the axis on the chassis to always stay in front of 
    // a plane passing through the middle of the axis on the wheel with normal
    // pointing inside the wheel.
    {
        auto p = cross(mrA, wheel_x) + cross(wheel_x, mdn);
        auto q = cross(mrB, wheel_x);

        auto &row = registry.get<constraint_row>(con.row[row_idx++]);
        row.J = {-wheel_x, -p, wheel_x, q};
        row.error = (dot(mdn, wheel_x) + 0.4) / dt;
        row.lower_limit = 0;
        row.upper_limit = large_scalar;
    }

    if (!steerable) {
        // Constrain wheel rotation axis to a plane that passes through upper pivot
        // on chassis with normal equals chassis' z axis
        auto q = cross(wheel_x, chassis_z);

        auto &row = registry.get<constraint_row>(con.row[row_idx++]);
        row.J = {vector3_zero, q, vector3_zero, -q};
        row.error = dot(wheel_x, chassis_z) / dt;
        row.lower_limit = -large_scalar;
        row.upper_limit = large_scalar;
    }
}

}