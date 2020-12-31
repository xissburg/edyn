#include "edyn/constraints/springdamper_constraint.hpp"
#include "edyn/comp/constraint.hpp"
#include "edyn/comp/constraint_row.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/util/spring_util.hpp"
#include "edyn/util/constraint_util.hpp"

namespace edyn {

void springdamper_constraint::init(entt::entity entity, constraint &con, entt::registry &registry) {
    for (size_t i = 0; i < 3; ++i) {
        add_constraint_row(entity, con, registry, 100);
    }
}

void springdamper_constraint::prepare(entt::entity, constraint &con, entt::registry &registry, scalar dt) {
    auto &posA = registry.get<position>(con.body[0]);
    auto &ornA = registry.get<orientation>(con.body[0]);
    auto rA = rotate(ornA, m_pivotA);
    auto pA = posA + rA;

    auto &posB = registry.get<position>(con.body[1]);
    auto &ornB = registry.get<orientation>(con.body[1]);
    auto rB = rotate(ornB, m_ctrl_arm_pivotB);
    auto pB = posB + rB;

    auto ctrl_armA = posA + rotate(ornA, m_ctrl_arm_pivotA);
    auto ctrl_armB = pB;
    auto ctrl_arm_dir = ctrl_armB - ctrl_armA;
    auto ctrl_arm_len = length(ctrl_arm_dir);
    ctrl_arm_dir /= ctrl_arm_len;

    auto chassis_z = rotate(ornA, vector3_z);
    auto n = cross(chassis_z, ctrl_arm_dir) * m_side;

    auto p = cross(rA, n);
    auto q = cross(rB, n);

    auto ctrl_arm_x = ctrl_arm_dir * m_side;
    auto ctrl_arm_y = cross(chassis_z, ctrl_arm_x);
    auto ctrl_arm_basis = matrix3x3_columns(ctrl_arm_x, ctrl_arm_y, chassis_z);
    auto ctrl_arm_pivot_rel = ctrl_arm_basis * m_ctrl_arm_pivot;
    auto coilover_dir = pA - (ctrl_armA + ctrl_arm_pivot_rel);
    auto distance = length(coilover_dir);
    coilover_dir /= distance;
    auto spring_len = distance - m_spring_offset - m_spring_perch_offset - m_damper_body_offset - m_spring_divider_length;
    auto error = spring_len - (m_spring_rest_length + m_second_spring_rest_length);

    auto inclination = std::abs(dot(n, coilover_dir));
    
    // Spring.
    {
        auto spring_impulse = m_stiffness_curve.get(-error) * inclination * dt;
        auto &row = registry.get<constraint_row>(con.row[0]);
        row.J = {n, p, -n, -q};
        row.error = spring_impulse > 0 ? -large_scalar : large_scalar;
        row.lower_limit = 0;
        row.upper_limit = std::max(scalar(0), spring_impulse);
    }

    // Damper.
    {
        auto &linvelA = registry.get<linvel>(con.body[0]);
        auto &angvelA = registry.get<angvel>(con.body[0]);
        auto &linvelB = registry.get<linvel>(con.body[1]);
        auto &angvelB = registry.get<angvel>(con.body[1]);

        auto velA = linvelA + cross(angvelA, rA);
        auto vel_ctrl_armA = linvelA + cross(angvelA, rotate(ornA, m_ctrl_arm_pivotA));
        auto vel_ctrl_armB = linvelB + cross(angvelB, rB);
        auto ratio = dot(ctrl_arm_pivot_rel, ctrl_arm_dir) / ctrl_arm_len;
        auto velB = vel_ctrl_armA * (1 - ratio) + vel_ctrl_armB * ratio;
        auto v_rel = velA - velB;
        auto speed = dot(coilover_dir, v_rel);
        auto damping_force = get_damping_force(speed) * inclination;
        auto impulse = std::abs(damping_force) * dt;

        auto &row = registry.get<constraint_row>(con.row[1]);
        row.J = {n, p, -n, -q};
        row.error = 0;
        row.lower_limit = -impulse;
        row.upper_limit =  impulse;

       // m_relspd = relspd;
    }

    // Damper piston limit when it fully extends.
    {
        auto error = distance - (m_piston_rod_length + m_damper_body_length + m_damper_body_offset);
        auto &row = registry.get<constraint_row>(con.row[2]);
        row.J = {n, p, -n, -q};
        row.error = error / dt;
        row.lower_limit = -large_scalar;
        row.upper_limit = 0;
    }
}

void springdamper_constraint::set_constant_spring_stiffness() {
    set_constant_spring_stiffness(m_spring_stiffness, m_spring_rest_length - m_spring_min_length);
}

void springdamper_constraint::set_constant_spring_stiffness(scalar stiffness, scalar max_defl) {
    m_stiffness_curve.clear();
    m_stiffness_curve.add(-1, 0);
    m_stiffness_curve.add(0, 0);
    m_stiffness_curve.add(max_defl, max_defl * stiffness);
    m_stiffness_curve.add(max_defl + 0.01, 1e6);
    m_stiffness_curve.add(max_defl + 1, 1e6);
}

void springdamper_constraint::set_dual_spring_stiffness() {
    set_dual_spring_stiffness(m_spring_stiffness, m_spring_rest_length - m_spring_min_length,
                              m_second_spring_stiffness, m_second_spring_rest_length - m_second_spring_min_length);
}

void springdamper_constraint::set_dual_spring_stiffness(scalar primary_stiffness, scalar primary_max_defl,
                                scalar secondary_stiffness, scalar secondary_max_defl) {
    m_stiffness_curve = spring_stiffness_curve(primary_stiffness, primary_max_defl, 
                                             secondary_stiffness, secondary_max_defl);
}

scalar springdamper_constraint::get_spring_deflection(const constraint &con, entt::registry &registry) const {
    auto &posA = registry.get<position>(con.body[0]);
    auto &ornA = registry.get<orientation>(con.body[0]);
    auto rA = rotate(ornA, m_pivotA);
    auto pA = posA + rA;

    auto &posB = registry.get<position>(con.body[1]);
    auto &ornB = registry.get<orientation>(con.body[1]);
    auto rB = rotate(ornB, m_ctrl_arm_pivotB);
    auto pB = posB + rB;

    auto ctrl_armA = posA + rotate(ornA, m_ctrl_arm_pivotA);
    auto ctrl_armB = pB;
    auto ctrl_arm_dir = ctrl_armA - ctrl_armB;
    auto ctrl_arm_len = length(ctrl_arm_dir);
    ctrl_arm_dir /= ctrl_arm_len;

    auto chassis_z = rotate(ornA, vector3_z);
    auto ctrl_arm_x = ctrl_arm_dir * m_side;
    auto ctrl_arm_y = cross(chassis_z, ctrl_arm_x);
    auto ctrl_arm_basis = matrix3x3_columns(ctrl_arm_x, ctrl_arm_y, chassis_z);
    auto ctrl_arm_pivot_rel = ctrl_arm_basis * m_ctrl_arm_pivot;
    auto coilover_dir = pA - (ctrl_armA + ctrl_arm_pivot_rel);
    auto distance = length(coilover_dir);
    auto spring_len = distance - m_spring_offset - m_spring_perch_offset - m_damper_body_offset - m_spring_divider_length;
    auto error = spring_len - (m_spring_rest_length + m_second_spring_rest_length);

    return error;
}

scalar springdamper_constraint::get_preload() const {
    return spring_preload(m_stiffness_curve,
                          m_piston_rod_length,
                          m_damper_body_length,
                          m_damper_body_offset,
                          m_spring_offset,
                          m_spring_perch_offset,
                          m_spring_divider_length,
                          m_spring_rest_length,
                          m_second_spring_rest_length);
}

scalar springdamper_constraint::get_combined_spring_stiffness() const {
    if (m_second_spring_stiffness > 0) {
        return m_spring_stiffness * m_second_spring_stiffness / (m_spring_stiffness + m_second_spring_stiffness);
    }

    return m_spring_stiffness;
}

vector3 springdamper_constraint::get_world_ctrl_arm_pivot(const constraint &con, entt::registry &registry) const {
    auto &posA = registry.get<position>(con.body[0]);
    auto &ornA = registry.get<orientation>(con.body[0]);

    auto &posB = registry.get<position>(con.body[1]);
    auto &ornB = registry.get<orientation>(con.body[1]);
    auto rB = rotate(ornB, m_ctrl_arm_pivotB);
    auto pB = posB + rB;

    auto ctrl_armA = posA + rotate(ornA, m_ctrl_arm_pivotA);
    auto ctrl_armB = pB;
    auto ctrl_arm_dir = ctrl_armA - ctrl_armB;
    auto ctrl_arm_len = length(ctrl_arm_dir);
    ctrl_arm_dir /= ctrl_arm_len;

    auto chassis_z = rotate(ornA, vector3_z);
    auto ctrl_arm_x = ctrl_arm_dir * m_side;
    auto ctrl_arm_y = cross(chassis_z, ctrl_arm_x);
    auto ctrl_arm_basis = matrix3x3_columns(ctrl_arm_x, ctrl_arm_y, chassis_z);
    auto ctrl_arm_pivot_rel = ctrl_arm_basis * m_ctrl_arm_pivot;
    auto ctrl_arm_pivot = ctrl_armA + ctrl_arm_pivot_rel;

    return ctrl_arm_pivot;
}

scalar springdamper_constraint::get_damping_force(scalar speed) const {
    if (speed < 0) {
        if (-speed > m_compression_knee_speed) {
            return m_slow_compression_damping * m_compression_knee_speed + 
                   m_fast_compression_damping * (-speed - m_compression_knee_speed);
        } else {
            return m_slow_compression_damping * -speed;
        }
    } else {
        if (speed > m_rebound_knee_speed) {
            return m_slow_rebound_damping * m_rebound_knee_speed + 
                   m_fast_rebound_damping * (speed - m_rebound_knee_speed);
        } else {
            return m_slow_rebound_damping * speed;
        }
    }
}

}