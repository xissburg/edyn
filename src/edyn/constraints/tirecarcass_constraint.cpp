#include "edyn/constraints/tirecarcass_constraint.hpp" 
#include "edyn/comp/constraint.hpp"
#include "edyn/comp/constraint_row.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/delta_linvel.hpp"
#include "edyn/comp/delta_angvel.hpp"
#include "edyn/comp/spin.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/util/constraint_util.hpp"
#include <entt/entt.hpp>

namespace edyn {

void tirecarcass_constraint::init(entt::entity entity, constraint &con, entt::registry &registry) {
    for (size_t i = 0; i < 10; ++i) {
        add_constraint_row(entity, con, registry, 100);
    }
}

void tirecarcass_constraint::prepare(entt::entity, constraint &con, 
                                     entt::registry &registry, scalar dt) {
    auto &posA    = registry.get<position   >(con.body[0]);
    auto &ornA    = registry.get<orientation>(con.body[0]);
    auto &linvelA = registry.get<linvel     >(con.body[0]);
    auto &angvelA = registry.get<angvel     >(con.body[0]);
    auto &spinA   = registry.get<spin       >(con.body[0]);
    auto &angleA  = registry.get<spin_angle >(con.body[0]);

    auto &posB    = registry.get<position   >(con.body[1]);
    auto &ornB    = registry.get<orientation>(con.body[1]);
    auto &linvelB = registry.get<linvel     >(con.body[1]);
    auto &angvelB = registry.get<angvel     >(con.body[1]);
    auto &spinB   = registry.get<spin       >(con.body[1]);
    auto &angleB  = registry.get<spin_angle >(con.body[1]);

    const auto axisA_x = quaternion_x(ornA);
    const auto axisA_y = quaternion_y(ornA);
    const auto axisA_z = quaternion_z(ornA);

    const auto axisB_x = quaternion_x(ornB);
    const auto axisB_y = quaternion_y(ornB);
    const auto axisB_z = quaternion_z(ornB);

    size_t idx = 0;

    // Lateral movement.
    {
        auto error = -dot(posA - posB, axisB_x);
        auto spring_force = error * m_lateral_stiffness;
        auto spring_impulse = spring_force * dt;

        auto &row = registry.get<constraint_row>(con.row[idx++]);
        row.J = {axisB_x, vector3_zero, -axisB_x, vector3_zero};
        row.error = spring_impulse > 0 ? -large_scalar : large_scalar;
        row.lower_limit = std::min(spring_impulse, scalar(0));
        row.upper_limit = std::max(scalar(0), spring_impulse);
    }

    // Lateral damping.
    {
        auto relspd = dot(linvelA - linvelB, axisB_x);
        auto damping_force = m_lateral_damping * relspd;
        auto damping_impulse = damping_force * dt;
        auto impulse = std::abs(damping_impulse);

        auto &row = registry.get<constraint_row>(con.row[idx++]);
        row.J = {axisB_x, vector3_zero, -axisB_x, vector3_zero};
        row.error = 0;
        row.lower_limit = -impulse;
        row.upper_limit =  impulse;

        m_lateral_relspd = relspd;
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
        auto error = -dot(axisB_x, axisA_z);
        auto spring_force = error * m_torsional_stiffness;
        auto spring_impulse = spring_force * dt;

        auto &row = registry.get<constraint_row>(con.row[idx++]);
        row.J = {vector3_zero, axisB_y, vector3_zero, -axisB_y};
        row.error = spring_impulse > 0 ? -large_scalar : large_scalar;
        row.lower_limit = std::min(spring_impulse, scalar(0));
        row.upper_limit = std::max(scalar(0), spring_impulse);
    }

    // Torsional damping.
    {
        auto relspd = dot(angvelA - angvelB, axisB_y);
        auto damping_force = m_torsional_damping * relspd;
        auto damping_impulse = damping_force * dt;
        auto impulse = std::abs(damping_impulse);

        auto &row = registry.get<constraint_row>(con.row[idx++]);
        row.J = {vector3_zero, axisB_y, vector3_zero, -axisB_y};
        row.error = 0;
        row.lower_limit = -impulse;
        row.upper_limit =  impulse;

        m_torsional_relspd = relspd;
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
        auto spring_torque = -error * m_longitudinal_stiffness;
        auto spring_impulse = spring_torque * dt;

        auto &row = registry.get<constraint_row>(con.row[idx++]);
        row.J = {vector3_zero, axisA_x, vector3_zero, -axisB_x};
        row.error = spring_impulse > 0 ? -large_scalar : large_scalar;
        row.lower_limit = std::min(spring_impulse, scalar(0));
        row.upper_limit = std::max(scalar(0), spring_impulse);
        row.use_spin[0] = true;
        row.use_spin[1] = true;
    }

    // Longitudinal damping.
    {
        auto relspd = spinA.s - spinB.s;
        auto damping_torque = m_longitudinal_damping * relspd;
        auto damping_impulse = damping_torque * dt;
        auto impulse = std::abs(damping_impulse);

        auto &row = registry.get<constraint_row>(con.row[idx++]);
        row.J = {vector3_zero, axisA_x, vector3_zero, -axisB_x};
        row.error = 0;
        row.lower_limit = -impulse;
        row.upper_limit =  impulse;
        row.use_spin[0] = true;
        row.use_spin[1] = true;

        m_longitudinal_relspd = relspd;
    }
}

void tirecarcass_constraint::iteration(entt::entity entity, constraint &con, 
                                       entt::registry &registry, scalar dt) {
    // Adjust damping row limits to account for velocity changes during iterations.
    auto &dvA = registry.get<delta_linvel>(con.body[0]);
    auto &dwA = registry.get<delta_angvel>(con.body[0]);
    auto &dsA = registry.get<delta_spin>(con.body[0]);
    auto &dvB = registry.get<delta_linvel>(con.body[1]);
    auto &dwB = registry.get<delta_angvel>(con.body[1]);
    auto &dsB = registry.get<delta_spin>(con.body[1]);

    // Lateral damping.
    {
        auto &row = registry.get<constraint_row>(con.row[1]);
        auto delta_relspd = dot(row.J[0], dvA) + 
                            dot(row.J[1], dwA) +
                            dot(row.J[2], dvB) +
                            dot(row.J[3], dwB);
        auto relspd = m_lateral_relspd + delta_relspd;
        auto damping_force = m_lateral_damping * relspd;
        auto damping_impulse = damping_force * dt;
        auto impulse = std::abs(damping_impulse);
        row.lower_limit = -impulse;
        row.upper_limit =  impulse;
    }

    // Torsional damping.
    {
        auto &row = registry.get<constraint_row>(con.row[6]);
        auto delta_relspd = dot(row.J[0], dvA) + 
                            dot(row.J[1], dwA) +
                            dot(row.J[2], dvB) +
                            dot(row.J[3], dwB);
        auto relspd = m_torsional_relspd + delta_relspd;
        auto damping_force = m_torsional_damping * relspd;
        auto damping_impulse = damping_force * dt;
        auto impulse = std::abs(damping_impulse);
        row.lower_limit = -impulse;
        row.upper_limit =  impulse;
    }

    // Longitudinal damping.
    {
        auto &row = registry.get<constraint_row>(con.row[9]);
        // Note the `+` because the impulse is equal and opposite.
        auto delta_relspd = dsA.s + dsB.s; 
        auto relspd = m_longitudinal_relspd + delta_relspd;
        auto damping_force = m_longitudinal_damping * relspd;
        auto damping_impulse = damping_force * dt;
        auto impulse = std::abs(damping_impulse);
        row.lower_limit = -impulse;
        row.upper_limit =  impulse;
    }
}

}