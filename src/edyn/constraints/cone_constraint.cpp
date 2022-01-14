#include "edyn/constraints/cone_constraint.hpp"
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
#include "edyn/dynamics/row_cache.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/util/constraint_util.hpp"
#include <entt/entity/registry.hpp>
#include <cmath>

namespace edyn {

template<>
void prepare_constraints<cone_constraint>(entt::registry &registry, row_cache &cache, scalar dt) {
    auto body_view = registry.view<position, orientation,
                                   linvel, angvel,
                                   mass_inv, inertia_world_inv,
                                   delta_linvel, delta_angvel>();
    auto con_view = registry.view<cone_constraint>();
    auto origin_view = registry.view<origin>();

    con_view.each([&] (cone_constraint &con) {
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
        const auto row_start_index = cache.rows.size();

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
            row.impulse = con.impulse[i];

            prepare_row(row, {}, linvelA, angvelA, linvelB, angvelB);
            warm_start(row);
        }

        auto axisA = con.frame[0].column(0);
        auto axisB_world = rotate(ornB, con.frame[1].column(0));

        // Transform a B's spin axis onto A's space so the angular span can
        // be interpolated.
        auto axisB_in_A = edyn::rotate(conjugate(ornA), axisB_world);
        vector3 torque_axis;
        scalar error;

        auto radius0 = std::sqrt(1 - edyn::square(con.span[0]));
        auto radius1 = std::sqrt(1 - edyn::square(con.span[1]));

        if (radius0 > radius1) {
            auto scaling = radius0 / radius1;
            auto axisB_in_A_scaled = con.frame[0] * normalize(axisB_in_A * con.frame[0] * vector3{1, 1, scaling});
            error = (dot(axisA, axisB_in_A_scaled) - con.span[0]) / dt;
            torque_axis = con.frame[0] * (cross(axisA, axisB_in_A_scaled) * con.frame[0] * vector3{1, 1, 1 / scaling});
        } else {
            auto scaling = radius1 / radius0;
            auto axisB_in_A_scaled = con.frame[0] * normalize((axisB_in_A * con.frame[0]) * vector3{1, scaling, 1});
            error = (dot(axisA, axisB_in_A_scaled) - con.span[1]) / dt;
            torque_axis = con.frame[0] * ((cross(axisA, axisB_in_A_scaled) * con.frame[0]) * vector3{1, 1 / scaling, 1});
        }

        torque_axis = rotate(ornA, torque_axis);

        /* auto error = (dot(axisA, axisB_in_A) - con.span[1]) / dt;
        auto torque_axis = rotate(ornA, cross(axisA, axisB_in_A)); */

        if (try_normalize(torque_axis)) {
            auto &row = cache.rows.emplace_back();
            row.J = {vector3_zero, torque_axis, vector3_zero, -torque_axis};
            row.inv_mA = inv_mA; row.inv_IA = inv_IA;
            row.inv_mB = inv_mB; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dwA = &dwA;
            row.dvB = &dvB; row.dwB = &dwB;
            row.lower_limit = 0;
            row.upper_limit = large_scalar;
            row.impulse = con.impulse[3];

            auto options = constraint_row_options{};
            options.error = error;
            options.restitution = con.limit_restitution;

            prepare_row(row, options, linvelA, angvelA, linvelB, angvelB);
            warm_start(row);
        }

        auto num_rows = cache.rows.size() - row_start_index;
        cache.con_num_rows.push_back(num_rows);
    });
}

template<>
bool solve_position_constraints<cone_constraint>(entt::registry &registry, scalar dt) {
    auto con_view = registry.view<cone_constraint>();
    auto body_view = registry.view<position, orientation, mass_inv, inertia_world_inv>();
    auto origin_view = registry.view<origin>();
    auto linear_error = scalar(0);
    auto angular_error = scalar(0);

    con_view.each([&] (cone_constraint &con) {
        auto [posA, ornA, inv_mA, inv_IA] =
            body_view.get<position, orientation, mass_inv, inertia_world_inv>(con.body[0]);
        auto [posB, ornB, inv_mB, inv_IB] =
            body_view.get<position, orientation, mass_inv, inertia_world_inv>(con.body[1]);

        auto originA = origin_view.contains(con.body[0]) ? origin_view.get<origin>(con.body[0]) : static_cast<vector3>(posA);
        auto originB = origin_view.contains(con.body[1]) ? origin_view.get<origin>(con.body[1]) : static_cast<vector3>(posB);

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

    if (linear_error < scalar(0.005) && angular_error < std::sin(to_radians(3))) {
        return true;
    }

    return false;
}

}
