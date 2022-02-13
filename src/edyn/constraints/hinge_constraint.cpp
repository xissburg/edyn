#include "edyn/constraints/hinge_constraint.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/math/math.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
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
#include "edyn/util/constraint_util.hpp"
#include <entt/entity/registry.hpp>
#include <cmath>

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

template<>
void prepare_constraints<hinge_constraint>(entt::registry &registry, row_cache &cache, scalar dt) {
    auto body_view = registry.view<position, orientation,
                                   linvel, angvel,
                                   mass_inv, inertia_world_inv,
                                   delta_linvel, delta_angvel>();
    auto con_view = registry.view<hinge_constraint>(entt::exclude_t<disabled_tag>{});
    auto origin_view = registry.view<origin>();

    con_view.each([&] (hinge_constraint &con) {
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
            row.lower_limit = -EDYN_SCALAR_MAX;
            row.upper_limit = EDYN_SCALAR_MAX;

            row.inv_mA = inv_mA; row.inv_IA = inv_IA;
            row.inv_mB = inv_mB; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dwA = &dwA;
            row.dvB = &dvB; row.dwB = &dwB;
            row.impulse = con.impulse[row_idx++];

            prepare_row(row, {}, linvelA, angvelA, linvelB, angvelB);
            warm_start(row);
        }

        // Make relative angular velocity go to zero along directions orthogonal
        // to the hinge axis where rotations are allowed.
        auto p = rotate(ornA, con.frame[0].column(1));
        auto q = rotate(ornA, con.frame[0].column(2));

        {
            auto &row = cache.rows.emplace_back();
            row.J = {vector3_zero, p, vector3_zero, -p};
            row.lower_limit = -EDYN_SCALAR_MAX;
            row.upper_limit = EDYN_SCALAR_MAX;

            row.inv_mA = inv_mA; row.inv_IA = inv_IA;
            row.inv_mB = inv_mB; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dwA = &dwA;
            row.dvB = &dvB; row.dwB = &dwB;
            row.impulse = con.impulse[row_idx++];

            prepare_row(row, {}, linvelA, angvelA, linvelB, angvelB);
            warm_start(row);
        }

        {
            auto &row = cache.rows.emplace_back();
            row.J = {vector3_zero, q, vector3_zero, -q};
            row.lower_limit = -EDYN_SCALAR_MAX;
            row.upper_limit = EDYN_SCALAR_MAX;

            row.inv_mA = inv_mA; row.inv_IA = inv_IA;
            row.inv_mB = inv_mB; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dwA = &dwA;
            row.dvB = &dvB; row.dwB = &dwB;
            row.impulse = con.impulse[row_idx++];

            prepare_row(row, {}, linvelA, angvelA, linvelB, angvelB);
            warm_start(row);
        }

        // Handle angular limits and friction.
        auto has_limit = con.angle_min < con.angle_max;
        auto has_spring = con.stiffness > 0;
        auto has_friction = con.friction_torque > 0 || con.damping > 0;
        vector3 hinge_axis;

        if (has_limit || has_spring || has_friction) {
            hinge_axis = rotate(ornA, con.frame[0].column(0));
        }

        if (has_limit || has_spring) {
            auto angle_axisB = rotate(ornB, con.frame[1].column(1));
            auto angle = std::atan2(dot(angle_axisB, q), dot(angle_axisB, p));
            auto previous_angle = normalize_angle(con.angle);
            // Find shortest path from previous angle to current in the [-π, π] range.
            auto angle_delta0 = angle - previous_angle;
            auto angle_delta1 = angle_delta0 + pi2 * to_sign(angle_delta0 < 0);
            auto angle_delta = std::abs(angle_delta0) < std::abs(angle_delta1) ? angle_delta0 : angle_delta1;
            con.angle += angle_delta;
        }

        if (has_limit) {
            // One row for angular limits.
            auto &row = cache.rows.emplace_back();
            row.J = {vector3_zero, hinge_axis, vector3_zero, -hinge_axis};
            row.inv_mA = inv_mA; row.inv_IA = inv_IA;
            row.inv_mB = inv_mB; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dwA = &dwA;
            row.dvB = &dvB; row.dwB = &dwB;
            row.impulse = con.impulse[row_idx++];

            auto limit_error = scalar{0};
            auto halfway_limit = (con.angle_min + con.angle_max) / scalar(2);

            // Set constraint limits according to which is the closer angular limit.
            if (con.angle < halfway_limit) {
                limit_error = con.angle_min - con.angle;
                row.lower_limit = -large_scalar;
                row.upper_limit = 0;
            } else {
                limit_error = con.angle_max - con.angle;
                row.lower_limit = 0;
                row.upper_limit = large_scalar;
            }

            auto options = constraint_row_options{};
            options.error = limit_error / dt;
            options.restitution = con.limit_restitution;

            prepare_row(row, options, linvelA, angvelA, linvelB, angvelB);
            warm_start(row);

            // Another row for bump stop spring.
            if (con.bump_stop_stiffness > 0 && con.bump_stop_angle > 0) {
                auto bump_stop_deflection = scalar{0};
                auto bump_stop_min = con.angle_min + con.bump_stop_angle;
                auto bump_stop_max = con.angle_max - con.bump_stop_angle;

                if (con.angle < bump_stop_min) {
                    bump_stop_deflection = con.angle - bump_stop_min;
                } else if (con.angle > bump_stop_max) {
                    bump_stop_deflection = con.angle - bump_stop_max;
                }

                if (bump_stop_deflection != 0) {
                    auto &row = cache.rows.emplace_back();
                    row.J = {vector3_zero, hinge_axis, vector3_zero, -hinge_axis};
                    row.inv_mA = inv_mA; row.inv_IA = inv_IA;
                    row.inv_mB = inv_mB; row.inv_IB = inv_IB;
                    row.dvA = &dvA; row.dwA = &dwA;
                    row.dvB = &dvB; row.dwB = &dwB;
                    row.impulse = con.impulse[row_idx++];

                    auto spring_force = con.bump_stop_stiffness * bump_stop_deflection;
                    auto spring_impulse = spring_force * dt;
                    row.lower_limit = std::min(spring_impulse, scalar(0));
                    row.upper_limit = std::max(scalar(0), spring_impulse);

                    auto options = constraint_row_options{};
                    options.error = -bump_stop_deflection / dt;

                    prepare_row(row, options, linvelA, angvelA, linvelB, angvelB);
                    warm_start(row);
                }
            }
        }

        if (has_spring) {
            auto &row = cache.rows.emplace_back();
            row.J = {vector3_zero, hinge_axis, vector3_zero, -hinge_axis};
            row.inv_mA = inv_mA; row.inv_IA = inv_IA;
            row.inv_mB = inv_mB; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dwA = &dwA;
            row.dvB = &dvB; row.dwB = &dwB;
            row.impulse = con.impulse[row_idx++];

            auto deflection = con.angle - con.rest_angle;
            auto spring_force = con.stiffness * deflection;
            auto spring_impulse = spring_force * dt;
            row.lower_limit = std::min(spring_impulse, scalar(0));
            row.upper_limit = std::max(scalar(0), spring_impulse);

            auto options = constraint_row_options{};
            options.error = -deflection / dt;

            prepare_row(row, options, linvelA, angvelA, linvelB, angvelB);
            warm_start(row);
        }

        if (has_friction) {
            // Since damping acts as a speed-dependent friction, a single row
            // is employed for both damping and constant friction.
            auto &row = cache.rows.emplace_back();
            row.J = {vector3_zero, hinge_axis, vector3_zero, -hinge_axis};
            row.inv_mA = inv_mA; row.inv_IA = inv_IA;
            row.inv_mB = inv_mB; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dwA = &dwA;
            row.dvB = &dvB; row.dwB = &dwB;
            row.impulse = con.impulse[row_idx++];

            auto friction_impulse = con.friction_torque * dt;

            if (con.damping > 0) {
                auto relvel = dot(angvelA, hinge_axis) - dot(angvelB, hinge_axis);
                friction_impulse += std::abs(relvel) * con.damping * dt;
            }

            row.lower_limit = -friction_impulse;
            row.upper_limit = friction_impulse;

            prepare_row(row, {}, linvelA, angvelA, linvelB, angvelB);
            warm_start(row);
        }

        cache.con_num_rows.push_back(row_idx);
    });
}

template<>
bool solve_position_constraints<hinge_constraint>(entt::registry &registry, scalar dt) {
    auto con_view = registry.view<hinge_constraint>();
    auto body_view = registry.view<position, orientation, mass_inv, inertia_world_inv>();
    auto origin_view = registry.view<origin>();
    auto linear_error = scalar(0);
    auto angular_error = scalar(0);

    con_view.each([&] (hinge_constraint &con) {
        auto [posA, ornA, inv_mA, inv_IA] = body_view.get(con.body[0]);
        auto [posB, ornB, inv_mB, inv_IB] = body_view.get(con.body[1]);

        auto originA = origin_view.contains(con.body[0]) ? origin_view.get<origin>(con.body[0]) : static_cast<vector3>(posA);
        auto originB = origin_view.contains(con.body[1]) ? origin_view.get<origin>(con.body[1]) : static_cast<vector3>(posB);

        auto axisA = rotate(ornA, con.frame[0].column(0));
        auto axisB = rotate(ornB, con.frame[1].column(0));

        // Apply angular correction first, with the goal of aligning the hinge axes.
        vector3 p, q;
        plane_space(axisA, p, q);
        auto u = cross(axisA, axisB);

        {
            auto J_invM_JT = dot(inv_IA * p, p) + dot(inv_IB * p, p);
            auto eff_mass = scalar(1) / J_invM_JT;
            auto error = dot(u, p);
            auto correction = error * eff_mass;
            ornA += quaternion_derivative(ornA, inv_IA * p * correction);
            ornB += quaternion_derivative(ornB, inv_IB * p * -correction);
            angular_error = std::max(std::abs(error), angular_error);
        }

        {
            auto J_invM_JT = dot(inv_IA * q, q) + dot(inv_IB * q, q);
            auto eff_mass = scalar(1) / J_invM_JT;
            auto error = dot(u, q);
            auto correction = error * eff_mass;
            ornA += quaternion_derivative(ornA, inv_IA * q * correction);
            ornB += quaternion_derivative(ornB, inv_IB * q * -correction);
            angular_error = std::max(std::abs(error), angular_error);
        }

        // Now apply another correction to join the pivot points together.
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

    if (linear_error < scalar(0.005) && angular_error < std::sin(to_radians(3))) {
        return true;
    }

    return false;
}

}
