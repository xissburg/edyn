#include "edyn/constraints/hinge_constraint.hpp"
#include "edyn/dynamics/position_solver.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/math/math.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/math/transform.hpp"
#include "edyn/dynamics/row_cache.hpp"

namespace edyn {

void hinge_constraint::set_axes(const vector3 &axisA, const vector3 &axisB) {
    vector3 p, q;
    plane_space(axisA, p, q);
    frame[0] = matrix3x3_columns(axisA, p, q);
    plane_space(axisB, p, q);
    frame[1] = matrix3x3_columns(axisB, p, q);
}

void hinge_constraint::reset_angle(const quaternion &ornA, const quaternion &ornB) {
    auto p = rotate(ornA, frame[0].column(1));
    auto q = rotate(ornA, frame[0].column(2));
    auto angle_axisB = rotate(ornB, frame[1].column(1));
    angle = std::atan2(dot(angle_axisB, q), dot(angle_axisB, p));
}

void hinge_constraint::prepare(
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
        row.lower_limit = -EDYN_SCALAR_MAX;
        row.upper_limit = EDYN_SCALAR_MAX;
        row.impulse = applied_impulse.linear[i];
    }

    // Make relative angular velocity go to zero along directions orthogonal
    // to the hinge axis where rotations are allowed.
    auto p = rotate(bodyA.orn, frame[0].column(1));
    auto q = rotate(bodyA.orn, frame[0].column(2));

    {
        auto &row = cache.add_row();
        row.J = {vector3_zero, p, vector3_zero, -p};
        row.lower_limit = -EDYN_SCALAR_MAX;
        row.upper_limit = EDYN_SCALAR_MAX;
        row.impulse = applied_impulse.hinge[0];
    }

    {
        auto &row = cache.add_row();
        row.J = {vector3_zero, q, vector3_zero, -q};
        row.lower_limit = -EDYN_SCALAR_MAX;
        row.upper_limit = EDYN_SCALAR_MAX;
        row.impulse = applied_impulse.hinge[1];
    }

    // Handle angular limits and friction.
    auto has_limit = angle_min < angle_max;
    auto has_spring = stiffness > 0;
    auto has_torque = torque > 0 || damping > 0;
    vector3 hinge_axis;

    if (has_limit || has_spring || has_torque) {
        hinge_axis = rotate(bodyA.orn, frame[0].column(0));
    }

    if (has_limit || has_spring) {
        auto angle_axisB = rotate(bodyB.orn, frame[1].column(1));
        auto current_angle = std::atan2(dot(angle_axisB, q), dot(angle_axisB, p));
        auto previous_angle = normalize_angle(angle);
        // Find shortest path from previous angle to current in the [-π, π] range.
        auto angle_delta0 = current_angle - previous_angle;
        auto angle_delta1 = angle_delta0 + pi2 * to_sign(angle_delta0 < 0);
        auto angle_delta = std::abs(angle_delta0) < std::abs(angle_delta1) ? angle_delta0 : angle_delta1;
        angle += angle_delta;
    }

    if (has_limit) {
        /* One row for angular limits. */ {
            auto &row = cache.add_row();
            row.J = {vector3_zero, hinge_axis, vector3_zero, -hinge_axis};
            row.impulse = applied_impulse.limit;

            auto limit_error = scalar{0};
            auto halfway_limit = (angle_min + angle_max) / scalar(2);

            // Set constraint limits according to which is the closer angular limit.
            if (angle < halfway_limit) {
                limit_error = angle_min - angle;
                row.lower_limit = -large_scalar;
                row.upper_limit = 0;
            } else {
                limit_error = angle_max - angle;
                row.lower_limit = 0;
                row.upper_limit = large_scalar;
            }

            auto &options = cache.get_options();
            options.error = limit_error / dt;
            options.restitution = limit_restitution;
        }

        // Another row for bump stop spring.
        if (bump_stop_stiffness > 0 && bump_stop_angle > 0) {
            auto bump_stop_deflection = scalar{0};
            auto bump_stop_min = angle_min + bump_stop_angle;
            auto bump_stop_max = angle_max - bump_stop_angle;

            if (angle < bump_stop_min) {
                bump_stop_deflection = angle - bump_stop_min;
            } else if (angle > bump_stop_max) {
                bump_stop_deflection = angle - bump_stop_max;
            }

            if (bump_stop_deflection != 0) {
                auto &row = cache.add_row();
                row.J = {vector3_zero, hinge_axis, vector3_zero, -hinge_axis};
                row.impulse = applied_impulse.bump_stop;
                auto spring_force = bump_stop_stiffness * bump_stop_deflection;
                auto spring_impulse = spring_force * dt;
                row.lower_limit = std::min(spring_impulse, scalar(0));
                row.upper_limit = std::max(scalar(0), spring_impulse);

                auto &options = cache.get_options();
                options.error = -bump_stop_deflection / dt;
            }
        }
    }

    if (has_spring) {
        auto &row = cache.add_row();
        row.J = {vector3_zero, hinge_axis, vector3_zero, -hinge_axis};
        row.impulse = applied_impulse.spring;

        auto deflection = angle - rest_angle;
        auto spring_force = stiffness * deflection;
        auto spring_impulse = spring_force * dt;
        row.lower_limit = std::min(spring_impulse, scalar(0));
        row.upper_limit = std::max(scalar(0), spring_impulse);

        auto &options = cache.get_options();
        options.error = -deflection / dt;
    }

    if (has_torque) {
        // Since damping acts as a speed-dependent friction, a single row
        // is employed for both damping and constant friction.
        auto &row = cache.add_row();
        row.J = {vector3_zero, hinge_axis, vector3_zero, -hinge_axis};
        row.impulse = applied_impulse.torque;

        auto torque_impulse = torque * dt;

        if (damping > 0) {
            auto relvel = dot(bodyA.angvel, hinge_axis) - dot(bodyB.angvel, hinge_axis);
            torque_impulse += std::abs(relvel) * damping * dt;
        }

        row.lower_limit = -torque_impulse;
        row.upper_limit = torque_impulse;

        auto &options = cache.get_options();
        options.error = -speed;
    }
}

void hinge_constraint::solve_position(position_solver &solver) {
    auto &posA = *solver.posA, &posB = *solver.posB;
    auto &ornA = *solver.ornA, &ornB = *solver.ornB;

    auto axisA = rotate(ornA, frame[0].column(0));
    auto axisB = rotate(ornB, frame[1].column(0));

    // Apply angular correction first, with the goal of aligning the hinge axes.
    vector3 p, q;
    plane_space(axisA, p, q);
    auto u = cross(axisA, axisB);

    if (auto error = dot(u, p); std::abs(error) > EDYN_EPSILON) {
        solver.solve({vector3_zero, p, vector3_zero, -p}, error);
    }

    if (auto error = dot(u, q); std::abs(error) > EDYN_EPSILON) {
        solver.solve({vector3_zero, q, vector3_zero, -q}, error);
    }

    // Now apply another correction to join the pivot points together.
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

void hinge_constraint::store_applied_impulses(const std::vector<scalar> &impulses) {
    unsigned row_idx = 0;

    for (int i = 0; i < 3; ++i) {
        applied_impulse.linear[i] = impulses[row_idx++];
    }

    for (int i = 0; i < 2; ++i) {
        applied_impulse.hinge[i] = impulses[row_idx++];
    }

    auto has_limit = angle_min < angle_max;
    auto has_spring = stiffness > 0;
    auto has_torque = torque > 0 || damping > 0;

    if (has_limit) {
        applied_impulse.limit = impulses[row_idx++];

        if (bump_stop_stiffness > 0 && bump_stop_angle > 0) {
            auto bump_stop_deflection = scalar{0};
            auto bump_stop_min = angle_min + bump_stop_angle;
            auto bump_stop_max = angle_max - bump_stop_angle;

            if (angle < bump_stop_min) {
                bump_stop_deflection = angle - bump_stop_min;
            } else if (angle > bump_stop_max) {
                bump_stop_deflection = angle - bump_stop_max;
            }

            if (bump_stop_deflection != 0) {
                applied_impulse.bump_stop = impulses[row_idx++];
            }
        }
    }

    if (has_spring) {
        applied_impulse.spring = impulses[row_idx++];
    }

    if (has_torque) {
        applied_impulse.torque = impulses[row_idx++];
    }
}

}
