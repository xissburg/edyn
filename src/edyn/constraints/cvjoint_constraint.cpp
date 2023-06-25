#include "edyn/constraints/cvjoint_constraint.hpp"
#include "edyn/dynamics/position_solver.hpp"
#include "edyn/math/math.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/math/transform.hpp"
#include "edyn/dynamics/row_cache.hpp"
#include "edyn/math/vector3.hpp"
#include <cmath>

namespace edyn {

void cvjoint_constraint::reset_angle(const quaternion &ornA, const quaternion &ornB) {
    twist_angle = relative_angle(ornA, ornB);
}

scalar cvjoint_constraint::relative_angle(const quaternion &ornA, const quaternion &ornB) const {
    auto twist_axisA = rotate(ornA, frame[0].column(0));
    auto twist_axisB = rotate(ornB, frame[1].column(0));
    return relative_angle(ornA, ornB, twist_axisA, twist_axisB);
}

scalar cvjoint_constraint::relative_angle(const quaternion &ornA, const quaternion &ornB,
                                          const vector3 &twist_axisA, const vector3 &twist_axisB) const {
    // Quaternion which rotates twist axis of B so it's parallel to the
    // twist axis of A.
    auto arc_quat = shortest_arc(twist_axisB, twist_axisA);

    // Transform a non-axial vector in the frame of B onto A's space so
    // the angular error can be calculated.
    auto angle_axisB = edyn::rotate(conjugate(ornA) * arc_quat * ornB, frame[1].column(1));
    return std::atan2(dot(angle_axisB, frame[0].column(2)),
                      dot(angle_axisB, frame[0].column(1)));
}

void cvjoint_constraint::update_angle(scalar new_angle) {
    auto previous_angle = normalize_angle(twist_angle);
    // Find shortest path from previous angle to current in
    // the [-π, π] range.
    auto angle_delta0 = new_angle - previous_angle;
    auto angle_delta1 = angle_delta0 + pi2 * to_sign(angle_delta0 < 0);
    auto angle_delta = std::abs(angle_delta0) < std::abs(angle_delta1) ? angle_delta0 : angle_delta1;
    twist_angle += angle_delta;
}

void cvjoint_constraint::prepare(
    const entt::registry &, entt::entity,
    constraint_row_prep_cache &cache, scalar dt,
    const constraint_body &bodyA, const constraint_body &bodyB) {

    auto pivotA = to_world_space(pivot[0], bodyA.origin, bodyA.orn);
    auto pivotB = to_world_space(pivot[1], bodyB.origin, bodyB.orn);
    auto rA = pivotA - bodyA.pos;
    auto rB = pivotB - bodyB.pos;

    const auto rA_skew = skew_matrix(rA);
    const auto rB_skew = skew_matrix(rB);
    constexpr auto I = matrix3x3_identity;

    // Make the position of pivot points match, akin to a `point_constraint`.
    for (int i = 0; i < 3; ++i) {
        auto &row = cache.add_row();
        row.J = {I.row[i], -rA_skew.row[i],
                -I.row[i],  rB_skew.row[i]};
        row.lower_limit = -large_scalar;
        row.upper_limit = large_scalar;
        row.impulse = applied_impulse.linear[i];
    }

    auto twist_axisA = rotate(bodyA.orn, frame[0].column(0));
    auto twist_axisB = rotate(bodyB.orn, frame[1].column(0));

    auto has_limit = twist_min < twist_max;

    // Relationship between velocity and relative angle along the twist axis.
    {
        auto &row = cache.add_row();
        row.J = {vector3_zero, twist_axisA, vector3_zero, -twist_axisB};
        row.impulse = applied_impulse.twist_limit;

        auto angle = relative_angle(bodyA.orn, bodyB.orn, twist_axisA, twist_axisB);
        auto &options = cache.get_options();

        if (has_limit) {
            update_angle(angle);

            auto limit_error = scalar{};
            const auto mid_angle = (twist_min + twist_max) / scalar(2);

            // Set constraint limits according to which is the closer
            // angular limit.
            if (twist_angle < mid_angle) {
                limit_error = twist_min - twist_angle;
                row.lower_limit = -large_scalar;
                row.upper_limit = 0;
            } else {
                limit_error = twist_max - twist_angle;
                row.lower_limit = 0;
                row.upper_limit = large_scalar;
            }

            // Only assign error if the limits haven't been violated. The
            // position constraints will fix angular limit errors later.
            if (twist_angle > twist_min && twist_angle < twist_max) {
                options.error = limit_error / dt;
            }

            options.restitution = twist_restitution;
        } else {
            row.lower_limit = -large_scalar;
            row.upper_limit = large_scalar;
        }
    }

    // Twist bump stops.
    if (has_limit && twist_bump_stop_stiffness > 0 && twist_bump_stop_angle > 0) {
        auto bump_stop_deflection = scalar{0};
        auto bump_stop_min = twist_min + twist_bump_stop_angle;
        auto bump_stop_max = twist_max - twist_bump_stop_angle;

        if (twist_angle < bump_stop_min) {
            bump_stop_deflection = twist_angle - bump_stop_min;
        } else if (twist_angle > bump_stop_max) {
            bump_stop_deflection = twist_angle - bump_stop_max;
        }

        auto &row = cache.add_row();
        row.J = {vector3_zero, twist_axisA, vector3_zero, -twist_axisB};
        row.impulse = applied_impulse.twist_bump_stop;

        auto spring_force = twist_bump_stop_stiffness * bump_stop_deflection;
        auto spring_impulse = spring_force * dt;
        row.lower_limit = std::min(spring_impulse, scalar(0));
        row.upper_limit = std::max(scalar(0), spring_impulse);

        auto &options = cache.get_options();
        options.error = -bump_stop_deflection / dt;
    }

    // Twist stiffness.
    if (has_limit && twist_stiffness > 0) {
        auto &row = cache.add_row();
        row.J = {vector3_zero, twist_axisA, vector3_zero, -twist_axisB};
        row.impulse = applied_impulse.twist_spring;

        auto deflection = twist_angle - twist_rest_angle;
        auto spring_force = twist_stiffness * deflection;
        auto spring_impulse = spring_force * dt;
        row.lower_limit = std::min(spring_impulse, scalar(0));
        row.upper_limit = std::max(scalar(0), spring_impulse);

        auto &options = cache.get_options();
        options.error = -deflection / dt;
    }

    // Twisting friction and damping.
    if (has_limit && (twist_friction_torque > 0 || twist_damping > 0)) {
        // Since damping acts as a speed-dependent friction, a single row
        // is employed for both damping and constant friction.
        auto &row = cache.add_row();
        row.J = {vector3_zero, twist_axisA, vector3_zero, -twist_axisB};
        row.impulse = applied_impulse.twist_friction_damping;

        auto friction_impulse = twist_friction_torque * dt;

        if (twist_damping > 0) {
            auto relvel = dot(bodyA.angvel, twist_axisA) - dot(bodyB.angvel, twist_axisB);
            friction_impulse += std::abs(relvel) * twist_damping * dt;
        }

        row.lower_limit = -friction_impulse;
        row.upper_limit = friction_impulse;
    }

    // Bending friction and damping.
    if (bend_friction_torque > 0 || bend_damping > 0) {
        // Apply friction and damping to slowdown the non-twisting
        // angular velocity.
        auto twist_angvelA = dot(bodyA.angvel, twist_axisA) * twist_axisA;
        auto twist_angvelB = dot(bodyB.angvel, twist_axisB) * twist_axisB;
        auto angvel_rel = (bodyA.angvel - twist_angvelA) - (bodyB.angvel - twist_angvelB);
        auto angspd_rel = length(angvel_rel);
        vector3 angvel_axis;

        if (angspd_rel > EDYN_EPSILON) {
            angvel_axis = angvel_rel / angspd_rel;
        } else {
            // Pick axis orthogonal to `twist_axisA`.
            angvel_axis = rotate(bodyA.orn, frame[0].column(1));
        }

        auto &row = cache.add_row();
        row.J = {vector3_zero, angvel_axis, vector3_zero, -angvel_axis};
        row.impulse = applied_impulse.bend_friction_damping;

        auto friction_impulse = bend_friction_torque * dt;

        if (twist_damping > 0) {
            friction_impulse += std::abs(angspd_rel) * bend_damping * dt;
        }

        row.lower_limit = -friction_impulse;
        row.upper_limit = friction_impulse;
    }

    // Bending spring.
    if (bend_stiffness > 0) {
        auto bend_axis = cross(rotate(bodyA.orn, rest_direction), twist_axisB);
        auto bend_axis_len = length(bend_axis);
        auto angle = std::asin(bend_axis_len);

        if (bend_axis_len > EDYN_EPSILON) {
            bend_axis /= bend_axis_len;
        } else {
            bend_axis = rotate(bodyA.orn, frame[0].column(1));
        }

        auto &row = cache.add_row();
        row.J = {vector3_zero, bend_axis, vector3_zero, -bend_axis};
        row.impulse = applied_impulse.bend_spring;

        auto spring_force = bend_stiffness * angle;
        auto spring_impulse = spring_force * dt;
        row.lower_limit = std::min(spring_impulse, scalar(0));
        row.upper_limit = std::max(scalar(0), spring_impulse);

        auto &options = cache.get_options();
        options.error = -angle / dt;
    }
}

void cvjoint_constraint::solve_position(position_solver &solver) {
    auto &posA = *solver.posA, &posB = *solver.posB;
    auto &ornA = *solver.ornA, &ornB = *solver.ornB;

    // Apply angular correction along twist axes.
    auto twist_axisA = rotate(ornA, frame[0].column(0));
    auto twist_axisB = rotate(ornB, frame[1].column(0));
    auto angle = relative_angle(ornA, ornB, twist_axisA, twist_axisB);
    auto has_limit = twist_min < twist_max;
    auto twist_error = scalar{};

    if (has_limit) {
        update_angle(angle);

        if (twist_angle < twist_min) {
            twist_error = twist_angle - twist_min;
        } else if (twist_angle > twist_max) {
            twist_error = twist_angle - twist_max;
        }
    } else {
        twist_error = angle;
    }

    solver.solve({vector3_zero, twist_axisA, vector3_zero, -twist_axisB}, twist_error);

    // Apply correction to join the pivot points together.
    auto originA = solver.get_originA(), originB = solver.get_originB();
    auto pivotA = to_world_space(pivot[0], originA, ornA);
    auto pivotB = to_world_space(pivot[1], originB, ornB);
    auto dir = pivotA - pivotB;
    auto error = length(dir);

    if (error > EDYN_EPSILON) {
        dir /= error;
        auto rA = pivotA - posA;
        auto rB = pivotB - posB;
        solver.solve({dir, cross(rA, dir), -dir, -cross(rB, dir)}, -error);
    }
}

void cvjoint_constraint::store_applied_impulses(const std::vector<scalar> &impulses) {
    unsigned row_idx = 0;

    for (int i = 0; i < 3; ++i) {
        applied_impulse.linear[i] = impulses[row_idx++];
    }

    applied_impulse.twist_limit = impulses[row_idx++];

    auto has_limit = twist_min < twist_max;

    if (has_limit && twist_bump_stop_stiffness > 0 && twist_bump_stop_angle > 0) {
        applied_impulse.twist_bump_stop = impulses[row_idx++];
    }

    if (has_limit && twist_stiffness > 0) {
        applied_impulse.twist_spring = impulses[row_idx++];
    }

    if (has_limit && (twist_friction_torque > 0 || twist_damping > 0)) {
        applied_impulse.twist_friction_damping = impulses[row_idx++];
    }

    if (bend_friction_torque > 0 || bend_damping > 0) {
        applied_impulse.bend_friction_damping = impulses[row_idx++];
    }

    if (bend_stiffness > 0) {
        applied_impulse.bend_spring = impulses[row_idx++];
    }
}

}
