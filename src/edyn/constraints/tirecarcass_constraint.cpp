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
    con.num_rows = 10;

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

    auto &posB    = registry.get<const position   >(rel.entity[1]);
    auto &ornB    = registry.get<const orientation>(rel.entity[1]);
    auto &linvelB = registry.get<const linvel     >(rel.entity[1]);
    auto &angvelB = registry.get<const angvel     >(rel.entity[1]);
    auto &spinB   = registry.get<const spin       >(rel.entity[1]);
    auto &angleB  = registry.get<const spin_angle >(rel.entity[1]);

    const auto axisA_x = quaternion_x(ornA);
    const auto axisA_y = quaternion_y(ornA);
    const auto axisA_z = quaternion_z(ornA);

    const auto axisB_x = quaternion_x(ornB);
    const auto axisB_y = quaternion_y(ornB);
    const auto axisB_z = quaternion_z(ornB);

    size_t idx = 0;

    // Lateral movement.
    {
        auto error = dot(posA - posB, axisB_x);
        auto vel = dot(linvelA - linvelB, axisB_x);
        auto force = std::abs(error * m_lateral_stiffness);
        auto impulse = force * dt;

        auto &row = registry.get<constraint_row>(con.row[idx++]);
        row.J = {axisB_x, vector3_zero, -axisB_x, vector3_zero};
        row.error = error / dt;
        row.lower_limit = -impulse;
        row.upper_limit = impulse;
    }

    // Lateral movement damping.
    {
        auto vel = dot(linvelA - linvelB, axisB_x);
        auto force = std::abs(vel * m_lateral_damping);
        auto impulse = force * dt;

        auto &row = registry.get<constraint_row>(con.row[idx++]);
        row.J = {axisB_x, vector3_zero, -axisB_x, vector3_zero};
        row.error = 0;
        row.lower_limit = -impulse;
        row.upper_limit = impulse;
    }

    // Prevent vertical movement.
    {
        auto &row = registry.get<constraint_row>(con.row[idx++]);
        row.J = {axisB_y, vector3_zero, -axisB_y, vector3_zero};
        row.error = dot(posA - posB, axisB_y) / dt;
        row.lower_limit = -large_scalar;
        row.upper_limit = large_scalar;
    }

    // Prevent backwards/forwards movement.
    {
        auto &row = registry.get<constraint_row>(con.row[idx++]);
        row.J = {axisB_z, vector3_zero, -axisB_z, vector3_zero};
        row.error = dot(posA - posB, axisB_z) / dt;
        row.lower_limit = -large_scalar;
        row.upper_limit = large_scalar;
    }

    // Prevent rotation along spin axis.
    {
        auto &row = registry.get<constraint_row>(con.row[idx++]);
        row.J = {vector3_zero, axisB_x, vector3_zero, -axisB_x};
        row.error = -dot(axisB_y, axisA_z) / dt;
        row.lower_limit = -large_scalar;
        row.upper_limit = large_scalar;
    }

    // Torsional.
    {
        auto error = dot(axisB_x, axisA_z);
        auto force = std::abs(error * m_torsional_stiffness);
        auto impulse = force * dt;

        auto &row = registry.get<constraint_row>(con.row[idx++]);
        row.J = {vector3_zero, axisB_y, vector3_zero, -axisB_y};
        row.error = error / dt;
        row.lower_limit = -impulse;
        row.upper_limit = impulse;
    }

    // Torsional damping.
    {
        auto vel = dot(angvelA - angvelB, axisB_y);
        auto force = std::abs(vel * m_torsional_damping);
        auto impulse = force * dt;

        auto &row = registry.get<constraint_row>(con.row[idx++]);
        row.J = {vector3_zero, axisB_y, vector3_zero, -axisB_y};
        row.error = 0;
        row.lower_limit = -impulse;
        row.upper_limit = impulse;
    }

    // Prevent rolling (rotation along forward axis).
    {
        auto &row = registry.get<constraint_row>(con.row[idx++]);
        row.J = {vector3_zero, axisB_z, vector3_zero, -axisB_z};
        row.error = -dot(axisB_x, axisA_y) / dt;
        row.lower_limit = -large_scalar;
        row.upper_limit = large_scalar;
    }

    // Longitudinal twist.
    {
        auto error = (angleA.s - angleB.s) + (angleA.count - angleB.count) * pi2;
        auto force = std::abs(error * m_longitudinal_stiffness);
        auto impulse = force * dt;

        auto &row = registry.get<constraint_row>(con.row[idx++]);
        row.J = {vector3_zero, axisA_x, vector3_zero, -axisB_x};
        row.error = error / dt;
        row.lower_limit = -impulse;
        row.upper_limit = impulse;
        row.use_spin[0] = true;
        row.use_spin[1] = true;
    }

    // Longitudinal twist damping.
    {
        auto vel = spinA - spinB;
        auto force = std::abs(vel * m_longitudinal_damping);
        auto impulse = force * dt;

        auto &row = registry.get<constraint_row>(con.row[idx++]);
        row.J = {vector3_zero, axisA_x, vector3_zero, -axisB_x};
        row.error = 0;
        row.lower_limit = -impulse;
        row.upper_limit = impulse;
        row.use_spin[0] = true;
        row.use_spin[1] = true;
    }
}

}