#include "edyn/constraints/tirecarcass_constraint.hpp" 
#include "edyn/comp/relation.hpp"
#include "edyn/comp/constraint.hpp"
#include "edyn/comp/constraint_row.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/spin.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/math/matrix3x3.hpp"
#include <entt/entt.hpp>

namespace edyn {

void tirecarcass_constraint::init(entt::entity, constraint &con, const relation &rel, entt::registry &registry) {
    con.num_rows = 7;

    for (size_t i = 0; i < con.num_rows; ++i) {
        auto e = registry.create();
        con.row[i] = e;
        auto &row = registry.assign<constraint_row>(e);
        row.entity = rel.entity;
        row.priority = 100;
    }
}

void tirecarcass_constraint::prepare(entt::entity, constraint &con, const relation &rel, entt::registry &registry, scalar dt) {
    auto &posA    = registry.get<const position   >(rel.entity[0]);
    auto &ornA    = registry.get<const orientation>(rel.entity[0]);
    auto &linvelA = registry.get<const linvel     >(rel.entity[0]);
    auto &angvelA = registry.get<const angvel     >(rel.entity[0]);
    auto &spinA   = registry.get<const spin       >(rel.entity[0]);
    auto &angleA  = registry.get<const spin_angle >(rel.entity[0]);
    auto spinvelA = rotate(ornA, vector3_x) * spinA;

    auto &posB    = registry.get<const position   >(rel.entity[1]);
    auto &ornB    = registry.get<const orientation>(rel.entity[1]);
    auto &linvelB = registry.get<const linvel     >(rel.entity[1]);
    auto &angvelB = registry.get<const angvel     >(rel.entity[1]);
    auto &spinB   = registry.get<const spin       >(rel.entity[1]);
    auto &angleB  = registry.get<const spin_angle >(rel.entity[1]);
    auto spinvelB = rotate(ornB, vector3_x) * spinB;

    // Lateral movement.
    {
        auto axis = rotate(ornB, vector3_x);
        auto error = dot(posA - posB, axis);
        auto vel = dot(linvelA - linvelB, axis);
        auto force = error * m_lateral_stiffness + vel * m_lateral_damping;
        auto impulse = std::abs(force) * dt;

        auto &row = registry.get<constraint_row>(con.row[0]);
        row.J = {axis, vector3_zero, -axis, vector3_zero};
        row.error = error / dt;
        row.lower_limit = -impulse;
        row.upper_limit = impulse;
    }

    // Prevent vertical movement.
    {
        auto axis = rotate(ornB, vector3_y);
        auto error = dot(posA - posB, axis);

        auto &row = registry.get<constraint_row>(con.row[1]);
        row.J = {axis, vector3_zero, -axis, vector3_zero};
        row.error = error / dt;
        row.lower_limit = -large_scalar;
        row.upper_limit = large_scalar;
    }

    // Prevent backwards/forwards movement.
    {
        auto axis = rotate(ornB, vector3_z);
        auto error = dot(posA - posB, axis);

        auto &row = registry.get<constraint_row>(con.row[2]);
        row.J = {axis, vector3_zero, -axis, vector3_zero};
        row.error = error / dt;
        row.lower_limit = -large_scalar;
        row.upper_limit = large_scalar;
    }

    // Longitudinal twist.
    {
        auto ornspinA = ornA * quaternion_axis_angle(vector3_x, angleA);
        auto ornspinB = ornB * quaternion_axis_angle(vector3_x, angleB);
        auto axis = rotate(ornspinA, vector3_x);
        auto axisB_y = rotate(ornspinB, vector3_y);
        auto axisB_y_proj = axisB_y - axis * dot(axisB_y, axis);
        axisB_y_proj = normalize(axisB_y_proj);

        auto axisA_z = rotate(ornspinA, vector3_z);
        auto error = -std::asin(dot(axisB_y_proj, axisA_z));
        auto vel = dot((angvelA + spinvelA) - (angvelB + spinvelB), axis);
        auto force = error * m_longitudinal_stiffness + vel * m_longitudinal_damping;
        auto impulse = std::abs(force) * dt;

        auto &row = registry.get<constraint_row>(con.row[3]);
        row.J = {vector3_zero, axis, vector3_zero, -axis};
        row.error = error / dt;
        row.lower_limit = -impulse;
        row.upper_limit = impulse;
        row.use_spin[0] = true;
        row.use_spin[1] = true;
    }

    // Prevent rolling (rotation along forward axis).
    {
        auto axis = rotate(ornB, vector3_x);
        auto axisA_y = rotate(ornA, vector3_y);
        auto axisA_y_proj = axisA_y - axis * dot(axisA_y, axis);
        axisA_y_proj = normalize(axisA_y_proj);

        auto axisB_z = rotate(ornB, vector3_z);
        auto error = std::asin(dot(axisA_y_proj, axisB_z));

        auto &row = registry.get<constraint_row>(con.row[4]);
        row.J = {vector3_zero, axis, vector3_zero, -axis};
        row.error = error / dt;
        row.lower_limit = -large_scalar;
        row.upper_limit = large_scalar;
    }

    // Torsional.
    {
        auto axis = rotate(ornB, vector3_y);
        auto axisA_x = rotate(ornA, vector3_x);
        auto axisA_x_proj = axisA_x - axis * dot(axisA_x, axis);
        axisA_x_proj = normalize(axisA_x_proj);

        auto axisB_z = rotate(ornB, vector3_z);
        auto error = std::asin(dot(axisA_x_proj, axisB_z));
        auto vel = dot(angvelA - angvelB, axis);
        auto force = error * m_torsional_stiffness + vel * m_torsional_damping;
        auto impulse = std::abs(force) * dt;

        auto &row = registry.get<constraint_row>(con.row[5]);
        row.J = {vector3_zero, axis, vector3_zero, -axis};
        row.error = error / dt;
        row.lower_limit = -impulse;
        row.upper_limit = impulse;
    }

    // Prevent rotation along spin axis.
    {
        auto axis = rotate(ornB, vector3_z);
        auto axisA_x = rotate(ornA, vector3_x);
        auto axisA_x_proj = axisA_x - axis * dot(axisA_x, axis);
        axisA_x_proj = normalize(axisA_x_proj);

        auto axisB_y = rotate(ornB, vector3_y);
        auto error = std::asin(dot(axisA_x_proj, axisB_y));

        auto &row = registry.get<constraint_row>(con.row[6]);
        row.J = {vector3_zero, axis, vector3_zero, -axis};
        row.error = error / dt;
        row.lower_limit = -large_scalar;
        row.upper_limit = large_scalar;
    }
}

}