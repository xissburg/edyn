#include "edyn/constraints/cvjoint_constraint.hpp"
#include "edyn/dynamics/position_solver.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/math/math.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/math/transform.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/delta_linvel.hpp"
#include "edyn/comp/delta_angvel.hpp"
#include "edyn/comp/origin.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/dynamics/row_cache.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/util/constraint_util.hpp"
#include <entt/entity/registry.hpp>
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

template<>
void prepare_constraint<cvjoint_constraint>(
    const entt::registry &, entt::entity, cvjoint_constraint &con,
    constraint_row_prep_cache &cache, scalar dt,
    const vector3 &originA, const vector3 &posA, const quaternion &ornA,
    const vector3 &linvelA, const vector3 &angvelA,
    scalar inv_mA, const matrix3x3 &inv_IA,
    const vector3 &originB, const vector3 &posB, const quaternion &ornB,
    const vector3 &linvelB, const vector3 &angvelB,
    scalar inv_mB, const matrix3x3 &inv_IB) {

    auto pivotA = to_world_space(con.pivot[0], originA, ornA);
    auto pivotB = to_world_space(con.pivot[1], originB, ornB);
    auto rA = pivotA - posA;
    auto rB = pivotB - posB;

    const auto rA_skew = skew_matrix(rA);
    const auto rB_skew = skew_matrix(rB);
    constexpr auto I = matrix3x3_identity;
    auto row_idx = size_t{};

    // Make the position of pivot points match, akin to a `point_constraint`.
    for (int i = 0; i < 3; ++i) {
        auto &row = cache.add_row();
        row.J = {I.row[i], -rA_skew.row[i],
                -I.row[i],  rB_skew.row[i]};
        row.lower_limit = -large_scalar;
        row.upper_limit = large_scalar;
        row.impulse = con.impulse[row_idx++];
    }

    auto twist_axisA = rotate(ornA, con.frame[0].column(0));
    auto twist_axisB = rotate(ornB, con.frame[1].column(0));

    auto has_limit = con.twist_min < con.twist_max;

    // Relationship between velocity and relative angle along the twist axis.
    {
        auto &row = cache.add_row();
        row.J = {vector3_zero, twist_axisA, vector3_zero, -twist_axisB};
        row.impulse = con.impulse[row_idx++];

        auto angle = con.relative_angle(ornA, ornB, twist_axisA, twist_axisB);
        auto &options = cache.get_options();

        if (has_limit) {
            con.update_angle(angle);

            auto limit_error = scalar{};
            const auto mid_angle = (con.twist_min + con.twist_max) / scalar(2);

            // Set constraint limits according to which is the closer
            // angular limit.
            if (con.twist_angle < mid_angle) {
                limit_error = con.twist_min - con.twist_angle;
                row.lower_limit = -large_scalar;
                row.upper_limit = 0;
            } else {
                limit_error = con.twist_max - con.twist_angle;
                row.lower_limit = 0;
                row.upper_limit = large_scalar;
            }

            // Only assign error if the limits haven't been violated. The
            // position constraints will fix angular limit errors later.
            if (con.twist_angle > con.twist_min && con.twist_angle < con.twist_max) {
                options.error = limit_error / dt;
            }

            options.restitution = con.twist_restitution;
        } else {
            row.lower_limit = -large_scalar;
            row.upper_limit = large_scalar;
        }
    }

    // Twist bump stops.
    if (has_limit && con.twist_bump_stop_stiffness > 0 && con.twist_bump_stop_angle > 0) {
        auto bump_stop_deflection = scalar{0};
        auto bump_stop_min = con.twist_min + con.twist_bump_stop_angle;
        auto bump_stop_max = con.twist_max - con.twist_bump_stop_angle;

        if (con.twist_angle < bump_stop_min) {
            bump_stop_deflection = con.twist_angle - bump_stop_min;
        } else if (con.twist_angle > bump_stop_max) {
            bump_stop_deflection = con.twist_angle - bump_stop_max;
        }

        auto &row = cache.add_row();
        row.J = {vector3_zero, twist_axisA, vector3_zero, -twist_axisB};
        row.impulse = con.impulse[row_idx++];

        auto spring_force = con.twist_bump_stop_stiffness * bump_stop_deflection;
        auto spring_impulse = spring_force * dt;
        row.lower_limit = std::min(spring_impulse, scalar(0));
        row.upper_limit = std::max(scalar(0), spring_impulse);

        auto &options = cache.get_options();
        options.error = -bump_stop_deflection / dt;
    }

    // Twist stiffness.
    if (has_limit && con.twist_stiffness > 0) {
        auto &row = cache.add_row();
        row.J = {vector3_zero, twist_axisA, vector3_zero, -twist_axisB};
        row.impulse = con.impulse[row_idx++];

        auto deflection = con.twist_angle - con.twist_rest_angle;
        auto spring_force = con.twist_stiffness * deflection;
        auto spring_impulse = spring_force * dt;
        row.lower_limit = std::min(spring_impulse, scalar(0));
        row.upper_limit = std::max(scalar(0), spring_impulse);

        auto &options = cache.get_options();
        options.error = -deflection / dt;
    }

    // Twisting friction and damping.
    if (has_limit && (con.twist_friction_torque > 0 || con.twist_damping > 0)) {
        // Since damping acts as a speed-dependent friction, a single row
        // is employed for both damping and constant friction.
        auto &row = cache.add_row();
        row.J = {vector3_zero, twist_axisA, vector3_zero, -twist_axisB};
        row.impulse = con.impulse[row_idx++];

        auto friction_impulse = con.twist_friction_torque * dt;

        if (con.twist_damping > 0) {
            auto relvel = dot(angvelA, twist_axisA) - dot(angvelB, twist_axisB);
            friction_impulse += std::abs(relvel) * con.twist_damping * dt;
        }

        row.lower_limit = -friction_impulse;
        row.upper_limit = friction_impulse;
    }

    // Bending friction and damping.
    if (con.bend_friction_torque > 0 || con.bend_damping > 0) {
        // Apply friction and damping to slowdown the non-twisting
        // angular velocity.
        auto twist_angvelA = dot(angvelA, twist_axisA) * twist_axisA;
        auto twist_angvelB = dot(angvelB, twist_axisB) * twist_axisB;
        auto angvel_rel = (angvelA - twist_angvelA) - (angvelB - twist_angvelB);
        auto angspd_rel = length(angvel_rel);
        vector3 angvel_axis;

        if (angspd_rel > EDYN_EPSILON) {
            angvel_axis = angvel_rel / angspd_rel;
        } else {
            // Pick axis orthogonal to `twist_axisA`.
            angvel_axis = rotate(ornA, con.frame[0].column(1));
        }

        auto &row = cache.add_row();
        row.J = {vector3_zero, angvel_axis, vector3_zero, -angvel_axis};
        row.impulse = con.impulse[row_idx++];

        auto friction_impulse = con.bend_friction_torque * dt;

        if (con.twist_damping > 0) {
            friction_impulse += std::abs(angspd_rel) * con.bend_damping * dt;
        }

        row.lower_limit = -friction_impulse;
        row.upper_limit = friction_impulse;
    }

    // Bending spring.
    if (con.bend_stiffness > 0) {
        auto bend_axis = cross(rotate(ornA, con.rest_direction), twist_axisB);
        auto bend_axis_len = length(bend_axis);
        auto angle = std::asin(bend_axis_len);

        if (bend_axis_len > EDYN_EPSILON) {
            bend_axis /= bend_axis_len;
        } else {
            bend_axis = rotate(ornA, con.frame[0].column(1));
        }

        auto &row = cache.add_row();
        row.J = {vector3_zero, bend_axis, vector3_zero, -bend_axis};
        row.impulse = con.impulse[row_idx++];

        auto spring_force = con.bend_stiffness * angle;
        auto spring_impulse = spring_force * dt;
        row.lower_limit = std::min(spring_impulse, scalar(0));
        row.upper_limit = std::max(scalar(0), spring_impulse);

        auto &options = cache.get_options();
        options.error = -angle / dt;
    }
}

template<>
void prepare_position_constraint<cvjoint_constraint>(
    entt::registry &registry, entt::entity entity, cvjoint_constraint &con,
    position_solver &solver) {

    auto originA = solver.get_originA(), originB = solver.get_originB();
    auto &posA = *solver.posA, &posB = *solver.posB;
    auto &ornA = *solver.ornA, &ornB = *solver.ornB;

    // Apply angular correction along twist axes.
    auto twist_axisA = rotate(ornA, con.frame[0].column(0));
    auto twist_axisB = rotate(ornB, con.frame[1].column(0));
    auto angle = con.relative_angle(ornA, ornB, twist_axisA, twist_axisB);
    auto has_limit = con.twist_min < con.twist_max;
    auto twist_error = scalar{};

    if (has_limit) {
        con.update_angle(angle);

        if (con.twist_angle < con.twist_min) {
            twist_error = con.twist_angle - con.twist_min;
        } else if (con.twist_angle > con.twist_max) {
            twist_error = con.twist_angle - con.twist_max;
        }
    } else {
        twist_error = angle;
    }

    solver.solve({vector3_zero, twist_axisA, vector3_zero, -twist_axisB}, twist_error);

    // Apply correction to join the pivot points together.
    auto pivotA = to_world_space(con.pivot[0], originA, ornA);
    auto pivotB = to_world_space(con.pivot[1], originB, ornB);
    auto dir = pivotA - pivotB;
    auto error = length(dir);

    if (error > EDYN_EPSILON) {
        dir /= error;
        auto rA = pivotA - posA;
        auto rB = pivotB - posB;
        solver.solve({dir, cross(rA, dir), -dir, -cross(rB, dir)}, -error);
    }
}

}
