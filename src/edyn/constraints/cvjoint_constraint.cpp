#include "edyn/constraints/cvjoint_constraint.hpp"
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
void prepare_constraints<cvjoint_constraint>(entt::registry &registry, row_cache &cache, scalar dt) {
    auto body_view = registry.view<position, orientation,
                                   linvel, angvel,
                                   mass_inv, inertia_world_inv,
                                   delta_linvel, delta_angvel>();
    auto con_view = registry.view<cvjoint_constraint>(entt::exclude_t<disabled_tag>{});
    auto origin_view = registry.view<origin>();

    con_view.each([&] (cvjoint_constraint &con) {
        auto [posA, ornA, linvelA, angvelA, inv_mA, inv_IA, dvA, dwA] = body_view.get(con.body[0]);
        auto [posB, ornB, linvelB, angvelB, inv_mB, inv_IB, dvB, dwB] = body_view.get(con.body[1]);

        auto originA = origin_view.contains(con.body[0]) ? origin_view.get<origin>(con.body[0]) : static_cast<vector3>(posA);
        auto originB = origin_view.contains(con.body[1]) ? origin_view.get<origin>(con.body[1]) : static_cast<vector3>(posB);

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
            auto &row = cache.rows.emplace_back();
            row.J = {I.row[i], -rA_skew.row[i],
                    -I.row[i],  rB_skew.row[i]};
            row.lower_limit = -large_scalar;
            row.upper_limit = large_scalar;

            row.inv_mA = inv_mA; row.inv_IA = inv_IA;
            row.inv_mB = inv_mB; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dwA = &dwA;
            row.dvB = &dvB; row.dwB = &dwB;
            row.impulse = con.impulse[row_idx++];

            prepare_row(row, {}, linvelA, angvelA, linvelB, angvelB);
            warm_start(row);
        }

        auto twist_axisA = rotate(ornA, con.frame[0].column(0));
        auto twist_axisB = rotate(ornB, con.frame[1].column(0));

        auto has_limit = con.twist_min < con.twist_max;

        // Relationship between velocity and relative angle along the twist axis.
        {
            auto &row = cache.rows.emplace_back();
            row.J = {vector3_zero, twist_axisA, vector3_zero, -twist_axisB};
            row.inv_mA = inv_mA; row.inv_IA = inv_IA;
            row.inv_mB = inv_mB; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dwA = &dwA;
            row.dvB = &dvB; row.dwB = &dwB;
            row.impulse = con.impulse[row_idx++];

            auto angle = con.relative_angle(ornA, ornB, twist_axisA, twist_axisB);
            auto options = constraint_row_options{};

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

            prepare_row(row, options, linvelA, angvelA, linvelB, angvelB);
            warm_start(row);
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

            auto &row = cache.rows.emplace_back();
            row.J = {vector3_zero, twist_axisA, vector3_zero, -twist_axisB};
            row.inv_mA = inv_mA; row.inv_IA = inv_IA;
            row.inv_mB = inv_mB; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dwA = &dwA;
            row.dvB = &dvB; row.dwB = &dwB;
            row.impulse = con.impulse[row_idx++];

            auto spring_force = con.twist_bump_stop_stiffness * bump_stop_deflection;
            auto spring_impulse = spring_force * dt;
            row.lower_limit = std::min(spring_impulse, scalar(0));
            row.upper_limit = std::max(scalar(0), spring_impulse);

            auto options = constraint_row_options{};
            options.error = -bump_stop_deflection / dt;

            prepare_row(row, options, linvelA, angvelA, linvelB, angvelB);
            warm_start(row);
        }

        // Twist stiffness.
        if (has_limit && con.twist_stiffness > 0) {
            auto &row = cache.rows.emplace_back();
            row.J = {vector3_zero, twist_axisA, vector3_zero, -twist_axisB};
            row.inv_mA = inv_mA; row.inv_IA = inv_IA;
            row.inv_mB = inv_mB; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dwA = &dwA;
            row.dvB = &dvB; row.dwB = &dwB;
            row.impulse = con.impulse[row_idx++];

            auto deflection = con.twist_angle - con.twist_rest_angle;
            auto spring_force = con.twist_stiffness * deflection;
            auto spring_impulse = spring_force * dt;
            row.lower_limit = std::min(spring_impulse, scalar(0));
            row.upper_limit = std::max(scalar(0), spring_impulse);

            auto options = constraint_row_options{};
            options.error = -deflection / dt;

            prepare_row(row, options, linvelA, angvelA, linvelB, angvelB);
            warm_start(row);
        }

        // Twisting friction and damping.
        if (has_limit && (con.twist_friction_torque > 0 || con.twist_damping > 0)) {
            // Since damping acts as a speed-dependent friction, a single row
            // is employed for both damping and constant friction.
            auto &row = cache.rows.emplace_back();
            row.J = {vector3_zero, twist_axisA, vector3_zero, -twist_axisB};
            row.inv_mA = inv_mA; row.inv_IA = inv_IA;
            row.inv_mB = inv_mB; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dwA = &dwA;
            row.dvB = &dvB; row.dwB = &dwB;
            row.impulse = con.impulse[row_idx++];

            auto friction_impulse = con.twist_friction_torque * dt;

            if (con.twist_damping > 0) {
                auto relvel = dot(angvelA, twist_axisA) - dot(angvelB, twist_axisB);
                friction_impulse += std::abs(relvel) * con.twist_damping * dt;
            }

            row.lower_limit = -friction_impulse;
            row.upper_limit = friction_impulse;

            prepare_row(row, {}, linvelA, angvelA, linvelB, angvelB);
            warm_start(row);
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

            auto &row = cache.rows.emplace_back();
            row.J = {vector3_zero, angvel_axis, vector3_zero, -angvel_axis};
            row.inv_mA = inv_mA; row.inv_IA = inv_IA;
            row.inv_mB = inv_mB; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dwA = &dwA;
            row.dvB = &dvB; row.dwB = &dwB;
            row.impulse = con.impulse[row_idx++];

            auto friction_impulse = con.bend_friction_torque * dt;

            if (con.twist_damping > 0) {
                friction_impulse += std::abs(angspd_rel) * con.bend_damping * dt;
            }

            row.lower_limit = -friction_impulse;
            row.upper_limit = friction_impulse;

            prepare_row(row, {}, linvelA, angvelA, linvelB, angvelB);
            warm_start(row);
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

            auto &row = cache.rows.emplace_back();
            row.J = {vector3_zero, bend_axis, vector3_zero, -bend_axis};
            row.inv_mA = inv_mA; row.inv_IA = inv_IA;
            row.inv_mB = inv_mB; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dwA = &dwA;
            row.dvB = &dvB; row.dwB = &dwB;
            row.impulse = con.impulse[row_idx++];

            auto spring_force = con.bend_stiffness * angle;
            auto spring_impulse = spring_force * dt;
            row.lower_limit = std::min(spring_impulse, scalar(0));
            row.upper_limit = std::max(scalar(0), spring_impulse);

            auto options = constraint_row_options{};
            options.error = -angle / dt;

            prepare_row(row, options, linvelA, angvelA, linvelB, angvelB);
            warm_start(row);
        }

        cache.con_num_rows.push_back(row_idx);
    });
}

template<>
bool solve_position_constraints<cvjoint_constraint>(entt::registry &registry, scalar dt) {
    auto con_view = registry.view<cvjoint_constraint>(entt::exclude_t<disabled_tag>{});
    auto body_view = registry.view<position, orientation, mass_inv, inertia_world_inv>();
    auto origin_view = registry.view<origin>();
    auto linear_error = scalar(0);
    auto angular_error = scalar(0);

    con_view.each([&] (cvjoint_constraint &con) {
        auto [posA, ornA, inv_mA, inv_IA] = body_view.get(con.body[0]);
        auto [posB, ornB, inv_mB, inv_IB] = body_view.get(con.body[1]);

        auto originA = origin_view.contains(con.body[0]) ? origin_view.get<origin>(con.body[0]) : static_cast<vector3>(posA);
        auto originB = origin_view.contains(con.body[1]) ? origin_view.get<origin>(con.body[1]) : static_cast<vector3>(posB);

        // Apply angular correction along twist axes.
        auto twist_axisA = rotate(ornA, con.frame[0].column(0));
        auto twist_axisB = rotate(ornB, con.frame[1].column(0));
        auto angle = con.relative_angle(ornA, ornB, twist_axisA, twist_axisB);
        auto J = std::array<vector3, 4>{vector3_zero, twist_axisA, vector3_zero, -twist_axisB};
        auto eff_mass = get_effective_mass(J, inv_mA, inv_IA, inv_mB, inv_IB);
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

        const auto twist_correction_factor = scalar(0.2);
        auto correction = twist_error * eff_mass * twist_correction_factor;
        ornA += quaternion_derivative(ornA, inv_IA * J[1] * correction);
        ornB += quaternion_derivative(ornB, inv_IB * J[3] * correction);
        angular_error = std::max(angle, angular_error);

        // Apply correction to join the pivot points together.
        auto pivotA = to_world_space(con.pivot[0], originA, ornA);
        auto pivotB = to_world_space(con.pivot[1], originB, ornB);
        auto dir = pivotA - pivotB;
        auto error = length(dir);

        if (error > EDYN_EPSILON) {
            dir /= error;

            auto rA = pivotA - posA;
            auto rB = pivotB - posB;
            auto J = std::array<vector3, 4>{dir, cross(rA, dir), -dir, -cross(rB, dir)};
            auto eff_mass = get_effective_mass(J, inv_mA, inv_IA, inv_mB, inv_IB);
            auto correction = -error * eff_mass;

            posA += inv_mA * J[0] * correction;
            posB += inv_mB * J[2] * correction;
            ornA += quaternion_derivative(ornA, inv_IA * J[1] * correction);
            ornB += quaternion_derivative(ornB, inv_IB * J[3] * correction);

            linear_error = std::max(error, linear_error);
        }

        ornA = normalize(ornA);
        ornB = normalize(ornB);

        auto basisA = to_matrix3x3(ornA);
        inv_IA = basisA * inv_IA * transpose(basisA);

        auto basisB = to_matrix3x3(ornB);
        inv_IB = basisB * inv_IB * transpose(basisB);
    });

    if (linear_error < scalar(0.005) && angular_error < to_radians(2)) {
        return true;
    }

    return false;
}

}