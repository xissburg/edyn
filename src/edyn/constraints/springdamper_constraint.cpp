#include "edyn/constraints/springdamper_constraint.hpp"
#include "edyn/comp/origin.hpp"
#include "edyn/config/config.h"
#include "edyn/constraints/constraint_row.hpp"
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
#include "edyn/math/constants.hpp"
#include "edyn/math/transform.hpp"
#include "edyn/util/spring_util.hpp"
#include "edyn/util/constraint_util.hpp"
#include "edyn/util/rigidbody.hpp"
#include "edyn/math/math.hpp"

namespace edyn {

void springdamper_constraint::prepare(
    const entt::registry &registry, entt::entity entity,
    constraint_row_prep_cache &cache, scalar dt,
    const constraint_body &bodyA, const constraint_body &bodyB) {

    auto ctrl_armA = to_world_space(m_ctrl_arm_pivotA, bodyA.origin, bodyA.orn);
    auto ctrl_armB = to_world_space(m_ctrl_arm_pivotB, bodyB.origin, bodyB.orn);

    auto ctrl_arm_dir = ctrl_armB - ctrl_armA;
    auto ctrl_arm_len = length(ctrl_arm_dir);
    ctrl_arm_dir /= ctrl_arm_len;

    auto rA = ctrl_armB - bodyA.pos;
    auto rB = ctrl_armB - bodyB.pos;

    scalar side = m_ctrl_arm_pivotA.x > 0 ? 1 : -1;
    auto chassis_z = rotate(bodyA.orn, vector3_z);
    auto ctrl_arm_x = ctrl_arm_dir * side;
    auto ctrl_arm_y = cross(chassis_z, ctrl_arm_x);
    auto ctrl_arm_basis = matrix3x3_columns(ctrl_arm_x, ctrl_arm_y, chassis_z);
    auto ctrl_arm_pivot_rel = ctrl_arm_basis * m_ctrl_arm_pivot;
    auto ctrl_arm_pivot = ctrl_armA + ctrl_arm_pivot_rel;
    auto coiloverA = to_world_space(m_pivotA, bodyA.origin, bodyA.orn);
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
    // (given by `m_ctrl_arm_pivot.x`) which then creates a force at the
    // wheel pivot which is inversely proportional to the control arm length.
    auto cos_theta = dot(d, coilover_dir);
    auto ctrl_arm_pivot_horizontal_dist = m_ctrl_arm_pivot.x * side;
    auto ctrl_arm_pivot_ratio = ctrl_arm_pivot_horizontal_dist / ctrl_arm_len;
    auto ctrl_arm_pivot_ratio_inv = scalar(1) / ctrl_arm_pivot_ratio;
    auto lever_term = ctrl_arm_pivot_ratio * cos_theta;

    // Spring.
    {
        auto &row = cache.add_row();
        row.J = {d, p, -d, -q};
        row.lower_limit = 0;
        row.impulse = applied_impulse.spring;

        const auto spring_room = coilover_len -
            (m_spring_offset + m_spring_perch_offset + m_damper_body_offset + m_spring_divider_length);

        const auto max_spring_deflection = m_spring_min_length + m_second_spring_min_length;

        if (spring_room < max_spring_deflection) {
            // Maximum deflection has been reached. Apply hard impulse.
            row.upper_limit = large_scalar;

            auto error = max_spring_deflection - spring_room;
            cache.get_options().error = -error * cos_theta * ctrl_arm_pivot_ratio_inv / dt;
        } else {
            auto rest_len = m_spring_rest_length + m_second_spring_rest_length;
            auto error = rest_len - spring_room;
            scalar spring_force;

            if (m_second_spring_stiffness > 0) {
                auto combined_stiffness = get_combined_spring_stiffness();
                auto second_max_defl = m_second_spring_rest_length - m_second_spring_min_length;

                // Find total deflection when secondary reaches its maximum deflection.
                // Create transition from combined stiffness to primary stiffness.
                auto transition_defl = second_max_defl * m_second_spring_stiffness / combined_stiffness;

                if (error < transition_defl) {
                    spring_force = std::max(error, edyn::scalar(0)) * combined_stiffness;
                } else {
                    spring_force = transition_defl * combined_stiffness + (error - transition_defl) * m_spring_stiffness;
                }
            } else {
                spring_force = std::max(error, edyn::scalar(0)) * m_spring_stiffness;
            }

            spring_force *= lever_term;

            auto spring_impulse = spring_force * dt;
            row.upper_limit = spring_impulse;

            // Make error inversely proportional to distance from control arm pivot.
            cache.get_options().error = -error * cos_theta * ctrl_arm_pivot_ratio_inv / dt;
        }
    }

    // Bump stop.
    {
        auto &row = cache.add_row();
        row.J = {d, p, -d, -q};
        row.lower_limit = 0;
        row.impulse = applied_impulse.bumpstop;

        const auto bumpstop_room = coilover_len -
            (m_spring_offset + m_damper_body_offset + m_damper_body_length);

        if (bumpstop_room < 0) {
            row.upper_limit = large_scalar;
            cache.get_options().error = bumpstop_room * cos_theta * ctrl_arm_pivot_ratio_inv / dt;
        } else {
            auto bumpstop_error = std::max(m_bumpstop_rest_length - bumpstop_room, edyn::scalar(0));
            auto bumpstop_force = bumpstop_error * m_bumpstop_stiffness * lever_term;
            auto spring_impulse = bumpstop_force * dt;
            row.upper_limit = spring_impulse;
            cache.get_options().error = -bumpstop_error * cos_theta * ctrl_arm_pivot_ratio_inv / dt;
        }
    }

    // Damper.
    {
        auto velA = bodyA.linvel + cross(bodyA.angvel, coiloverA - bodyA.pos);
        auto vel_ctrl_armA = bodyA.linvel + cross(bodyA.angvel, ctrl_armA - bodyA.pos);
        auto vel_ctrl_armB = bodyB.linvel + cross(bodyB.angvel, rB);
        auto velB = lerp(vel_ctrl_armA, vel_ctrl_armB, ctrl_arm_pivot_ratio);
        auto v_rel = velA - velB;
        auto speed = dot(coilover_dir, v_rel);
        auto damping_force = get_damping_force(speed) * lever_term;
        auto damping_impulse = std::abs(damping_force) * dt;

        auto &row = cache.add_row();
        row.J = {d, p, -d, -q};
        row.lower_limit = -damping_impulse;
        row.upper_limit =  damping_impulse;
        row.impulse = applied_impulse.damper;
    }

    // Damper piston limit when it fully extends.
    {
        auto &row = cache.add_row();
        row.J = {d, p, -d, -q};
        row.lower_limit = -large_scalar;
        row.upper_limit = 0;
        row.impulse = applied_impulse.damper_limit;

        auto max_coilover_len = m_piston_rod_length + m_damper_body_length + m_damper_body_offset;
        auto limit_error = max_coilover_len - coilover_len;

        // Coilover has extended beyond limit. Apply reverse impulse.
        cache.get_options().error = -limit_error * cos_theta * ctrl_arm_pivot_ratio_inv / dt;
    }
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
    // TODO
    return {};
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

void springdamper_constraint::store_applied_impulses(const std::vector<scalar> &impulses) {
    unsigned row_idx = 0;
    applied_impulse.spring = impulses[row_idx++];
    applied_impulse.bumpstop = impulses[row_idx++];
    applied_impulse.damper = impulses[row_idx++];
    applied_impulse.damper_limit = impulses[row_idx++];
}

}
