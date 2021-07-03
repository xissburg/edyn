#include "edyn/constraints/springdamper_constraint.hpp"
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
#include "edyn/util/spring_util.hpp"
#include "edyn/util/constraint_util.hpp"

namespace edyn {

template<>
void prepare_constraints<springdamper_constraint>(entt::registry &registry, row_cache &cache, scalar dt) {
    auto body_view = registry.view<position, orientation,
                                   linvel, angvel,
                                   mass_inv, inertia_world_inv,
                                   delta_linvel, delta_angvel>();
    auto con_view = registry.view<springdamper_constraint>();
    auto imp_view = registry.view<constraint_impulse>();

    con_view.each([&] (entt::entity entity, springdamper_constraint &con) {
        auto [posA, ornA, linvelA, angvelA, inv_mA, inv_IA, dvA, dwA] =
            body_view.get<position, orientation, linvel, angvel, mass_inv, inertia_world_inv, delta_linvel, delta_angvel>(con.body[0]);
        auto [posB, ornB, linvelB, angvelB, inv_mB, inv_IB, dvB, dwB] =
            body_view.get<position, orientation, linvel, angvel, mass_inv, inertia_world_inv, delta_linvel, delta_angvel>(con.body[1]);
        auto &imp = imp_view.get(entity);

        auto rA = rotate(ornA, con.m_pivotA);
        auto pA = posA + rA;
        auto rB = rotate(ornB, con.m_ctrl_arm_pivotB);
        auto pB = posB + rB;

        auto ctrl_armA = posA + rotate(ornA, con.m_ctrl_arm_pivotA);
        auto ctrl_armB = pB;
        auto ctrl_arm_dir = ctrl_armB - ctrl_armA;
        auto ctrl_arm_len = length(ctrl_arm_dir);
        ctrl_arm_dir /= ctrl_arm_len;

        auto chassis_z = rotate(ornA, vector3_z);
        auto n = cross(chassis_z, ctrl_arm_dir) * con.m_side;

        auto p = cross(rA, n);
        auto q = cross(rB, n);

        auto ctrl_arm_x = ctrl_arm_dir * con.m_side;
        auto ctrl_arm_y = cross(chassis_z, ctrl_arm_x);
        auto ctrl_arm_basis = matrix3x3_columns(ctrl_arm_x, ctrl_arm_y, chassis_z);
        auto ctrl_arm_pivot_rel = ctrl_arm_basis * con.m_ctrl_arm_pivot;
        auto coilover_dir = pA - (ctrl_armA + ctrl_arm_pivot_rel);
        auto distance = length(coilover_dir);
        coilover_dir /= distance;
        auto spring_len = distance - con.m_spring_offset - con.m_spring_perch_offset - con.m_damper_body_offset - con.m_spring_divider_length;
        auto error = spring_len - (con.m_spring_rest_length + con.m_second_spring_rest_length);

        auto inclination = std::abs(dot(n, coilover_dir));

        // Spring.
        {
            auto spring_impulse = con.m_stiffness_curve.get(-error) * inclination * dt;
            auto &row = cache.rows.emplace_back();
            row.J = {n, p, -n, -q};
            row.lower_limit = 0;
            row.upper_limit = std::max(scalar(0), spring_impulse);

            auto options = constraint_row_options{};
            options.error = spring_impulse > 0 ? -large_scalar : large_scalar;

            row.inv_mA = inv_mA; row.inv_IA = inv_IA;
            row.inv_mB = inv_mB; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dwA = &dwA;
            row.dvB = &dvB; row.dwB = &dwB;
            row.impulse = imp.values[0];

            prepare_row(row, options, linvelA, linvelB, angvelA, angvelB);
            warm_start(row);
        }

        // Damper.
        {
            auto &linvelA = registry.get<linvel>(con.body[0]);
            auto &angvelA = registry.get<angvel>(con.body[0]);
            auto &linvelB = registry.get<linvel>(con.body[1]);
            auto &angvelB = registry.get<angvel>(con.body[1]);

            auto velA = linvelA + cross(angvelA, rA);
            auto vel_ctrl_armA = linvelA + cross(angvelA, rotate(ornA, con.m_ctrl_arm_pivotA));
            auto vel_ctrl_armB = linvelB + cross(angvelB, rB);
            auto ratio = dot(ctrl_arm_pivot_rel, ctrl_arm_dir) / ctrl_arm_len;
            auto velB = vel_ctrl_armA * (1 - ratio) + vel_ctrl_armB * ratio;
            auto v_rel = velA - velB;
            auto speed = dot(coilover_dir, v_rel);
            auto damping_force = con.get_damping_force(speed) * inclination;
            auto impulse = std::abs(damping_force) * dt;

            auto &row = cache.rows.emplace_back();
            row.J = {n, p, -n, -q};
            row.lower_limit = -impulse;
            row.upper_limit =  impulse;

            row.inv_mA = inv_mA; row.inv_IA = inv_IA;
            row.inv_mB = inv_mB; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dwA = &dwA;
            row.dvB = &dvB; row.dwB = &dwB;
            row.impulse = imp.values[1];

            prepare_row(row, {}, linvelA, linvelB, angvelA, angvelB);
            warm_start(row);

            // con.m_relspd = relspd;
        }

        // Damper piston limit when it fully extends.
        {
            auto error = distance - (con.m_piston_rod_length + con.m_damper_body_length + con.m_damper_body_offset);
            auto &row = cache.rows.emplace_back();
            row.J = {n, p, -n, -q};
            row.lower_limit = -large_scalar;
            row.upper_limit = 0;

            auto options = constraint_row_options{};
            options.error = error / dt;

            row.inv_mA = inv_mA; row.inv_IA = inv_IA;
            row.inv_mB = inv_mB; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dwA = &dwA;
            row.dvB = &dvB; row.dwB = &dwB;
            row.impulse = imp.values[2];

            prepare_row(row, options, linvelA, linvelB, angvelA, angvelB);
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

scalar springdamper_constraint::get_spring_deflection(entt::registry &registry) const {
    auto &posA = registry.get<position>(body[0]);
    auto &ornA = registry.get<orientation>(body[0]);
    auto rA = rotate(ornA, m_pivotA);
    auto pA = posA + rA;

    auto &posB = registry.get<position>(body[1]);
    auto &ornB = registry.get<orientation>(body[1]);
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

vector3 springdamper_constraint::get_world_ctrl_arm_pivot(entt::registry &registry) const {
    auto &posA = registry.get<position>(body[0]);
    auto &ornA = registry.get<orientation>(body[0]);

    auto &posB = registry.get<position>(body[1]);
    auto &ornB = registry.get<orientation>(body[1]);
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
