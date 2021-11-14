#include "edyn/constraints/springdamper_constraint.hpp"
#include "edyn/comp/origin.hpp"
#include "edyn/constraints/constraint_row.hpp"
#include "edyn/constraints/constraint_impulse.hpp"
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
#include "edyn/math/transform.hpp"
#include "edyn/util/spring_util.hpp"
#include "edyn/util/constraint_util.hpp"
#include "edyn/util/rigidbody.hpp"
#include "edyn/math/math.hpp"

namespace edyn {

template<>
void prepare_constraints<springdamper_constraint>(entt::registry &registry, row_cache &cache, scalar dt) {
    auto body_view = registry.view<position, orientation,
                                   linvel, angvel,
                                   mass_inv, inertia_world_inv,
                                   delta_linvel, delta_angvel>();
    auto con_view = registry.view<springdamper_constraint>();
    auto imp_view = registry.view<constraint_impulse>();
    auto origin_view = registry.view<origin>();

    con_view.each([&] (entt::entity entity, springdamper_constraint &con) {
        auto [posA, ornA, linvelA, angvelA, inv_mA, inv_IA, dvA, dwA] =
            body_view.get<position, orientation, linvel, angvel, mass_inv, inertia_world_inv, delta_linvel, delta_angvel>(con.body[0]);
        auto [posB, ornB, linvelB, angvelB, inv_mB, inv_IB, dvB, dwB] =
            body_view.get<position, orientation, linvel, angvel, mass_inv, inertia_world_inv, delta_linvel, delta_angvel>(con.body[1]);
        auto &imp = imp_view.get<constraint_impulse>(entity);

        scalar side = con.m_ctrl_arm_pivotA.x > 0 ? 1 : -1;

        auto originA = origin_view.contains(con.body[0]) ? origin_view.get<origin>(con.body[0]) : static_cast<vector3>(posA);
        auto originB = origin_view.contains(con.body[1]) ? origin_view.get<origin>(con.body[1]) : static_cast<vector3>(posB);

        auto ctrl_armA = to_world_space(con.m_ctrl_arm_pivotA, originA, ornA);
        auto ctrl_armB = to_world_space(con.m_ctrl_arm_pivotB, originB, ornB);

        auto ctrl_arm_dir = ctrl_armB - ctrl_armA;
        auto ctrl_arm_len = length(ctrl_arm_dir);
        ctrl_arm_dir /= ctrl_arm_len;

        auto rA = ctrl_armB - posA;
        auto rB = ctrl_armB - posB;

        auto chassis_z = rotate(ornA, vector3_z);
        auto ctrl_arm_x = ctrl_arm_dir * side;
        auto ctrl_arm_y = cross(chassis_z, ctrl_arm_x);
        auto ctrl_arm_basis = matrix3x3_columns(ctrl_arm_x, ctrl_arm_y, chassis_z);
        auto ctrl_arm_pivot_rel = ctrl_arm_basis * con.m_ctrl_arm_pivot;
        auto ctrl_arm_pivot = ctrl_armA + ctrl_arm_pivot_rel;
        auto coiloverA = to_world_space(con.m_pivotA, originA, ornA);
        auto coilover_dir = coiloverA - ctrl_arm_pivot;
        auto coilover_len = length(coilover_dir);
        coilover_dir /= coilover_len;

        // Apply corrective impulse at the wheel pivot along the direction
        // normal to the control arm.
        auto d = ctrl_arm_y;
        auto p = cross(rA, d);
        auto q = cross(rB, d);

        // Account for angle between the coilover and the control arm normal and
        // the lever created by the length of the control arm. The force is applied
        // somewhere in the middle of the control arm, which generates a torque
        // proportional to the distance from the control arm pivot on the chassis
        // (given by `con.m_ctrl_arm_pivot.x`) which then creates a force at the
        // wheel pivot which is inversely proportional to the control arm length.
        auto cos_theta = dot(d, coilover_dir);
        auto ctrl_arm_pivot_horizontal_dist = con.m_ctrl_arm_pivot.x * side;
        auto ctrl_arm_pivot_ratio = ctrl_arm_pivot_horizontal_dist / ctrl_arm_len;
        auto ctrl_arm_pivot_ratio_inv = scalar(1) / ctrl_arm_pivot_ratio;
        auto lever_term = ctrl_arm_pivot_ratio * cos_theta;

        // Spring.
        {
            auto spring_len = coilover_len - con.m_spring_offset - con.m_spring_perch_offset - con.m_damper_body_offset - con.m_spring_divider_length;
            auto rest_len = con.m_spring_rest_length + con.m_second_spring_rest_length;
            auto error = rest_len - spring_len;
            auto spring_force = con.m_stiffness_curve.get(error) * lever_term;
            auto spring_impulse = spring_force * dt;
            auto &row = cache.rows.emplace_back();
            row.J = {d, p, -d, -q};
            row.lower_limit = 0;
            row.upper_limit = spring_impulse;

            auto options = constraint_row_options{};
            // Make error inversely proportional to distance from control arm pivot.
            options.error = -error * cos_theta * ctrl_arm_pivot_ratio_inv / dt;

            row.inv_mA = inv_mA; row.inv_IA = inv_IA;
            row.inv_mB = inv_mB; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dwA = &dwA;
            row.dvB = &dvB; row.dwB = &dwB;
            row.impulse = imp.values[0];

            prepare_row(row, options, linvelA, angvelA, linvelB, angvelB);
            warm_start(row);
        }

        // Damper.
        {
            auto &linvelA = registry.get<linvel>(con.body[0]);
            auto &angvelA = registry.get<angvel>(con.body[0]);
            auto &linvelB = registry.get<linvel>(con.body[1]);
            auto &angvelB = registry.get<angvel>(con.body[1]);

            auto velA = linvelA + cross(angvelA, coiloverA - posA);
            auto vel_ctrl_armA = linvelA + cross(angvelA, ctrl_armA - posA);
            auto vel_ctrl_armB = linvelB + cross(angvelB, rB);
            auto velB = lerp(vel_ctrl_armA, vel_ctrl_armB, ctrl_arm_pivot_ratio);
            auto v_rel = velA - velB;
            auto speed = dot(coilover_dir, v_rel);
            auto damping_force = con.get_damping_force(speed) * lever_term;
            auto impulse = std::abs(damping_force) * dt;

            auto &row = cache.rows.emplace_back();
            row.J = {d, p, -d, -q};
            row.lower_limit = -impulse;
            row.upper_limit =  impulse;

            row.inv_mA = inv_mA; row.inv_IA = inv_IA;
            row.inv_mB = inv_mB; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dwA = &dwA;
            row.dvB = &dvB; row.dwB = &dwB;
            row.impulse = imp.values[1];

            prepare_row(row, {}, linvelA, angvelA, linvelB, angvelB);
            warm_start(row);
        }

        // Damper piston limit when it fully extends.
        {
            auto &row = cache.rows.emplace_back();
            row.J = {d, p, -d, -q};
            row.lower_limit = -large_scalar;
            row.upper_limit = 0;

            auto max_coilover_len = con.m_piston_rod_length + con.m_damper_body_length + con.m_damper_body_offset;
            auto limit_error = max_coilover_len - coilover_len;
            auto options = constraint_row_options{};

            // Coilover has extended beyond limit. Apply reverse impulse.
            options.error = -limit_error * cos_theta * ctrl_arm_pivot_ratio_inv / dt;

            row.inv_mA = inv_mA; row.inv_IA = inv_IA;
            row.inv_mB = inv_mB; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dwA = &dwA;
            row.dvB = &dvB; row.dwB = &dwB;
            row.impulse = imp.values[2];

            prepare_row(row, options, linvelA, angvelA, linvelB, angvelB);
            warm_start(row);
        }

        cache.con_num_rows.push_back(3);
    });
}

void springdamper_constraint::set_constant_spring_stiffness() {
    set_constant_spring_stiffness(m_spring_stiffness, m_spring_rest_length - m_spring_min_length);
}

void springdamper_constraint::set_constant_spring_stiffness(scalar stiffness, scalar max_defl) {
    m_stiffness_curve.clear();
    m_stiffness_curve.add(-1, 0);
    m_stiffness_curve.add(0, 0);
    m_stiffness_curve.add(max_defl, max_defl * stiffness);
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

scalar springdamper_constraint::get_spring_deflection(entt::registry &registry) const {
    auto posA = edyn::get_rigidbody_origin(registry, body[0]);
    auto ornA = registry.get<orientation>(body[0]);

    auto posB = edyn::get_rigidbody_origin(registry, body[1]);
    auto ornB = registry.get<orientation>(body[1]);

    auto pivotA = edyn::to_world_space(m_pivotA, posA, ornA);
    auto ctrl_armA = edyn::to_world_space(m_ctrl_arm_pivotA, posA, ornA);
    auto ctrl_armB = edyn::to_world_space(m_ctrl_arm_pivotB, posB, ornB);
    auto ctrl_arm_dir = ctrl_armA - ctrl_armB;
    auto ctrl_arm_len = length(ctrl_arm_dir);
    ctrl_arm_dir /= ctrl_arm_len;

    scalar side = m_ctrl_arm_pivotA.x > 0 ? 1 : -1;
    auto chassis_z = rotate(ornA, vector3_z);
    auto ctrl_arm_x = ctrl_arm_dir * side;
    auto ctrl_arm_y = cross(chassis_z, ctrl_arm_x);
    auto ctrl_arm_basis = matrix3x3_columns(ctrl_arm_x, ctrl_arm_y, chassis_z);
    auto ctrl_arm_pivot_rel = ctrl_arm_basis * m_ctrl_arm_pivot;
    auto coilover_dir = pivotA - (ctrl_armA + ctrl_arm_pivot_rel);
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

vector3 springdamper_constraint::get_world_ctrl_arm_pivot(entt::registry &registry) const {
    auto posA = edyn::get_rigidbody_origin(registry, body[0]);
    auto ornA = registry.get<orientation>(body[0]);

    auto posB = edyn::get_rigidbody_origin(registry, body[1]);
    auto ornB = registry.get<orientation>(body[1]);

    auto ctrl_armA = edyn::to_world_space(m_ctrl_arm_pivotA, posA, ornA);
    auto ctrl_armB = edyn::to_world_space(m_ctrl_arm_pivotB, posB, ornB);
    auto ctrl_arm_dir = ctrl_armA - ctrl_armB;
    auto ctrl_arm_len = length(ctrl_arm_dir);
    ctrl_arm_dir /= ctrl_arm_len;

    scalar side = m_ctrl_arm_pivotA.x > 0 ? 1 : -1;
    auto chassis_z = rotate(ornA, vector3_z);
    auto ctrl_arm_x = ctrl_arm_dir * side;
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
