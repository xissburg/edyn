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
#include "edyn/dynamics/row_cache.hpp"
#include "edyn/util/constraint_util.hpp"
#include <entt/entity/registry.hpp>
#include <cmath>

namespace edyn {

template<>
void prepare_constraints<cvjoint_constraint>(entt::registry &registry, row_cache &cache, scalar dt) {
    auto body_view = registry.view<position, orientation,
                                   linvel, angvel,
                                   mass_inv, inertia_world_inv,
                                   delta_linvel, delta_angvel>();
    auto con_view = registry.view<cvjoint_constraint>();
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
        const auto row_start_index = cache.rows.size();

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
            row.impulse = con.impulse[i];

            prepare_row(row, {}, linvelA, angvelA, linvelB, angvelB);
            warm_start(row);
        }

        {
            auto spin_axisA = quaternion_x(ornA);
            auto spin_axisB = quaternion_x(ornB);

            auto &row = cache.rows.emplace_back();
            row.J = {vector3_zero, spin_axisA, vector3_zero, -spin_axisB};
            row.lower_limit = -EDYN_SCALAR_MAX;
            row.upper_limit = EDYN_SCALAR_MAX;

            row.inv_mA = inv_mA; row.inv_IA = inv_IA;
            row.inv_mB = inv_mB; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dwA = &dwA;
            row.dvB = &dvB; row.dwB = &dwB;
            row.impulse = con.impulse[3];

            // Quaternion which rotates spin axis of B so it's parallel to the
            // spin axis of A.
            auto arc_quat = shortest_arc(spin_axisB, spin_axisA);

            // Transform a non-axial vector in the frame of B onto A's space so
            // the angular error can be calculated.
            auto angle_axisB = edyn::rotate(conjugate(ornA) * arc_quat * ornB, con.frame[1].column(1));
            auto angle = std::atan2(dot(angle_axisB, con.frame[0].column(2)),
                                    dot(angle_axisB, con.frame[0].column(1)));

            auto options = constraint_row_options{};
            options.error = -angle / dt;

            prepare_row(row, options, linvelA, angvelA, linvelB, angvelB);
            warm_start(row);
        }

        auto num_rows = cache.rows.size() - row_start_index;
        cache.con_num_rows.push_back(num_rows);
    });
}

template<>
bool solve_position_constraints<cvjoint_constraint>(entt::registry &registry, scalar dt) {
    auto con_view = registry.view<cvjoint_constraint>();
    auto body_view = registry.view<position, orientation, mass_inv, inertia_world_inv>();
    auto origin_view = registry.view<origin>();
    auto linear_error = scalar(0);
    auto angular_error = scalar(0);

    con_view.each([&] (cvjoint_constraint &con) {
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