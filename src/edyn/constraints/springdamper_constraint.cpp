#include "edyn/constraints/springdamper_constraint.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/dynamics/row_cache.hpp"
#include "edyn/math/math.hpp"
#include "edyn/math/transform.hpp"
#include "edyn/util/rigidbody.hpp"

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

    scalar side = m_ctrl_arm_pivotA.x > 0 ? 1 : -1;
    // Use chassis z axis as the control arm rotation axis. Could be made
    // arbitrary in the future.
    auto ctrl_arm_z = rotate(bodyA.orn, vector3_z);
    auto ctrl_arm_x = ctrl_arm_dir * side;
    auto ctrl_arm_y = cross(ctrl_arm_z, ctrl_arm_x);
    auto ctrl_arm_basis = matrix3x3_columns(ctrl_arm_x, ctrl_arm_y, ctrl_arm_z);
    auto ctrl_arm_pivot_rel = ctrl_arm_basis * m_ctrl_arm_pivot;
    auto ctrl_arm_pivot = ctrl_armA + ctrl_arm_pivot_rel;
    auto coiloverA = to_world_space(m_pivotA, bodyA.origin, bodyA.orn);
    auto coilover_dir = coiloverA - ctrl_arm_pivot;
    auto coilover_len = length(coilover_dir);
    coilover_dir /= coilover_len;

    // Apply corrective impulse at the wheel pivot along the direction
    // normal to the control arm.
    auto d = ctrl_arm_y;
    auto rA = ctrl_armB - bodyA.pos;
    auto rB = ctrl_armB - bodyB.pos;
    auto p = cross(rA, d);
    auto q = cross(rB, d);

    // The coilover applies force on the control arm at `ctrl_arm_pivot`
    // which generates a torque proportional to the distance to `ctrl_armA`
    // projected on the plane of control arm rotation, where the normal
    // vector is `ctrl_arm_z`.
    // This torque will produce a force at the wheel pivot `ctrl_armB` which
    // is inversely proportional to the control arm length.
    // Calculate cosine of angle between coilover axis and direction of
    // tangential component of force.
    auto tangent_force_dir = normalize(cross(ctrl_arm_z, ctrl_arm_pivot_rel)) * side;
    auto cos_theta = dot(coilover_dir, tangent_force_dir);
    auto ctrl_arm_pivot_proj_len = std::sqrt(m_ctrl_arm_pivot.x * m_ctrl_arm_pivot.x + m_ctrl_arm_pivot.y * m_ctrl_arm_pivot.y);
    auto ctrl_arm_pivot_ratio = ctrl_arm_pivot_proj_len / ctrl_arm_len;
    auto ctrl_arm_pivot_ratio_inv = scalar(1) / ctrl_arm_pivot_ratio;
    // The motion ratio represents how much the coilover compresses/extends
    // proportionally to the movement of the control arm pivot at the wheel.
    auto motion_ratio = ctrl_arm_pivot_ratio * cos_theta;

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
                    spring_force = std::max(error, scalar(0)) * combined_stiffness;
                } else {
                    spring_force = transition_defl * combined_stiffness + (error - transition_defl) * m_spring_stiffness;
                }
            } else {
                spring_force = std::max(error, scalar(0)) * m_spring_stiffness;
            }

            spring_force *= motion_ratio;

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
            auto bumpstop_error = std::max(m_bumpstop_rest_length - bumpstop_room, scalar(0));
            auto bumpstop_force = bumpstop_error * m_bumpstop_stiffness * motion_ratio;
            auto spring_impulse = bumpstop_force * dt;
            row.upper_limit = spring_impulse;
            cache.get_options().error = -bumpstop_error * cos_theta * ctrl_arm_pivot_ratio_inv / dt;
        }
    }

    // Damper.
    {
        // Calculate angular velocity of control arm.
        auto vel_ctrl_armA = bodyA.linvel + cross(bodyA.angvel, ctrl_armA - bodyA.pos);
        auto vel_ctrl_armB = bodyB.linvel + cross(bodyB.angvel, ctrl_armB - bodyB.pos);
        // Resultant velocity at control arm pivot on wheel.
        auto vel_rel_ctrl_arm = project_direction(vel_ctrl_armB - vel_ctrl_armA, ctrl_arm_z);
        auto ang_vel_sign = dot(vel_rel_ctrl_arm, ctrl_arm_y) * side > 0 ? 1: -1;
        // Angular velocity is linear velocity divided by radius.
        auto ang_spd_ctrl_arm = length(vel_rel_ctrl_arm) / ctrl_arm_len;
        auto ang_vel_ctrl_arm = ctrl_arm_z * (ang_spd_ctrl_arm * ang_vel_sign);

        // Velocity of coilover pivot on chassis.
        auto velA = bodyA.linvel + cross(bodyA.angvel, coiloverA - bodyA.pos);
        // Velocity of coilover pivot on control arm.
        auto velB = vel_ctrl_armA + cross(ang_vel_ctrl_arm, ctrl_arm_pivot_rel);
        auto v_rel = velA - velB;
        auto speed = dot(coilover_dir, v_rel);
        auto damping_force = get_damping_force(speed) * motion_ratio;
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
    auto posA = get_rigidbody_origin(registry, body[0]);
    auto ornA = registry.get<orientation>(body[0]);

    auto posB = get_rigidbody_origin(registry, body[1]);
    auto ornB = registry.get<orientation>(body[1]);

    auto pivotA = to_world_space(m_pivotA, posA, ornA);
    auto ctrl_armA = to_world_space(m_ctrl_arm_pivotA, posA, ornA);
    auto ctrl_armB = to_world_space(m_ctrl_arm_pivotB, posB, ornB);
    auto ctrl_arm_dir = ctrl_armA - ctrl_armB;
    auto ctrl_arm_len = length(ctrl_arm_dir);
    ctrl_arm_dir /= ctrl_arm_len;

    scalar side = m_ctrl_arm_pivotA.x > 0 ? 1 : -1;
    auto ctrl_arm_z = rotate(ornA, vector3_z);
    auto ctrl_arm_x = ctrl_arm_dir * side;
    auto ctrl_arm_y = cross(ctrl_arm_z, ctrl_arm_x);
    auto ctrl_arm_basis = matrix3x3_columns(ctrl_arm_x, ctrl_arm_y, ctrl_arm_z);
    auto ctrl_arm_pivot_rel = ctrl_arm_basis * m_ctrl_arm_pivot;
    auto ctrl_arm_pivot = ctrl_armA + ctrl_arm_pivot_rel;
    auto coilover_dir = pivotA - ctrl_arm_pivot;
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
    auto posA = get_rigidbody_origin(registry, body[0]);
    auto ornA = registry.get<orientation>(body[0]);

    auto posB = get_rigidbody_origin(registry, body[1]);
    auto ornB = registry.get<orientation>(body[1]);

    auto ctrl_armA = to_world_space(m_ctrl_arm_pivotA, posA, ornA);
    auto ctrl_armB = to_world_space(m_ctrl_arm_pivotB, posB, ornB);
    auto ctrl_arm_dir = ctrl_armA - ctrl_armB;
    auto ctrl_arm_len = length(ctrl_arm_dir);
    ctrl_arm_dir /= ctrl_arm_len;

    scalar side = m_ctrl_arm_pivotA.x > 0 ? 1 : -1;
    auto ctrl_arm_z = rotate(ornA, vector3_z);
    auto ctrl_arm_x = ctrl_arm_dir * side;
    auto ctrl_arm_y = cross(ctrl_arm_z, ctrl_arm_x);
    auto ctrl_arm_basis = matrix3x3_columns(ctrl_arm_x, ctrl_arm_y, ctrl_arm_z);
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

scalar springdamper_constraint::get_relative_speed(entt::registry &registry) const {
    auto posA = get_rigidbody_origin(registry, body[0]);
    auto ornA = registry.get<orientation>(body[0]);

    auto posB = get_rigidbody_origin(registry, body[1]);
    auto ornB = registry.get<orientation>(body[1]);

    auto ctrl_armA = to_world_space(m_ctrl_arm_pivotA, posA, ornA);
    auto ctrl_armB = to_world_space(m_ctrl_arm_pivotB, posB, ornB);
    auto ctrl_arm_dir = ctrl_armA - ctrl_armB;
    auto ctrl_arm_len = length(ctrl_arm_dir);
    ctrl_arm_dir /= ctrl_arm_len;

    // Build control arm basis to calculate world space pivot location.
    scalar side = m_ctrl_arm_pivotA.x > 0 ? 1 : -1;
    auto ctrl_arm_x = ctrl_arm_dir * side;
    auto ctrl_arm_z = rotate(ornA, vector3_z);
    auto ctrl_arm_y = cross(ctrl_arm_z, ctrl_arm_x);
    auto ctrl_arm_basis = matrix3x3_columns(ctrl_arm_x, ctrl_arm_y, ctrl_arm_z);
    auto ctrl_arm_pivot_rel = ctrl_arm_basis * m_ctrl_arm_pivot;
    auto ctrl_arm_pivot = ctrl_armA + ctrl_arm_pivot_rel;

    auto &linvelA = registry.get<linvel>(body[0]);
    auto &angvelA = registry.get<angvel>(body[0]);
    auto &linvelB = registry.get<linvel>(body[1]);
    auto &angvelB = registry.get<angvel>(body[1]);

    // Calculate angular velocity of control arm.
    auto vel_ctrl_armA = linvelA + cross(angvelA, ctrl_armA - posA);
    auto vel_ctrl_armB = linvelB + cross(angvelB, ctrl_armB - posB);
    // Resultant velocity at control arm pivot on wheel.
    auto vel_rel_ctrl_arm = project_direction(vel_ctrl_armB - vel_ctrl_armA, ctrl_arm_z);
    auto ang_vel_sign = dot(vel_rel_ctrl_arm, ctrl_arm_y) * side > 0 ? 1: -1;
    // Angular velocity is linear velocity divided by radius.
    auto ang_spd_ctrl_arm = length(vel_rel_ctrl_arm) / ctrl_arm_len;
    auto ang_vel_ctrl_arm = ctrl_arm_z * (ang_spd_ctrl_arm * ang_vel_sign);

    auto coiloverA = to_world_space(m_pivotA, posA, ornA);
    auto coilover_dir = coiloverA - ctrl_arm_pivot;
    auto coilover_len = length(coilover_dir);
    coilover_dir /= coilover_len;

    // Velocity of coilover pivot on chassis.
    auto velA = linvelA + cross(angvelA, coiloverA - posA);
    // Velocity of coilover pivot on control arm.
    auto velB = vel_ctrl_armA + cross(ang_vel_ctrl_arm, ctrl_arm_pivot_rel);
    auto v_rel = velA - velB;
    auto speed = dot(coilover_dir, v_rel);

    return speed;
}

void springdamper_constraint::store_applied_impulses(const std::vector<scalar> &impulses) {
    unsigned row_idx = 0;
    applied_impulse.spring = impulses[row_idx++];
    applied_impulse.bumpstop = impulses[row_idx++];
    applied_impulse.damper = impulses[row_idx++];
    applied_impulse.damper_limit = impulses[row_idx++];
}

}
