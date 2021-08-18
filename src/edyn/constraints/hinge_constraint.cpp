#include "edyn/constraints/hinge_constraint.hpp"
#include "edyn/constraints/constraint_impulse.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/math/math.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/delta_linvel.hpp"
#include "edyn/comp/delta_angvel.hpp"
#include "edyn/comp/center_of_mass.hpp"
#include "edyn/dynamics/row_cache.hpp"
#include "edyn/util/constraint_util.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

template<>
void prepare_constraints<hinge_constraint>(entt::registry &registry, row_cache &cache, scalar dt) {
    auto body_view = registry.view<position, orientation,
                                   linvel, angvel,
                                   mass_inv, inertia_world_inv,
                                   delta_linvel, delta_angvel>();
    auto con_view = registry.view<hinge_constraint, constraint_impulse>();
    auto com_view = registry.view<center_of_mass>();

    con_view.each([&] (hinge_constraint &con, constraint_impulse &imp) {
        auto [posA, ornA, linvelA, angvelA, inv_mA, inv_IA, dvA, dwA] =
            body_view.get<position, orientation, linvel, angvel, mass_inv, inertia_world_inv, delta_linvel, delta_angvel>(con.body[0]);
        auto [posB, ornB, linvelB, angvelB, inv_mB, inv_IB, dvB, dwB] =
            body_view.get<position, orientation, linvel, angvel, mass_inv, inertia_world_inv, delta_linvel, delta_angvel>(con.body[1]);

        auto originA = static_cast<vector3>(posA);
        auto originB = static_cast<vector3>(posB);

        if (com_view.contains(con.body[0])) {
            auto &com = com_view.get(con.body[0]);
            originA = to_world_space(-com, posA, ornA);
        }

        if (com_view.contains(con.body[1])) {
            auto &com = com_view.get(con.body[1]);
            originB = to_world_space(-com, posB, ornB);
        }

        auto pivotA = to_world_space(con.pivot[0], originA, ornA);
        auto pivotB = to_world_space(con.pivot[1], originB, ornB);
        auto rA = pivotA - posA;
        auto rB = pivotB - posB;

        const auto rA_skew = skew_matrix(rA);
        const auto rB_skew = skew_matrix(rB);
        constexpr auto I = matrix3x3_identity;
        size_t row_idx = 0;

        for (; row_idx < 3; ++row_idx) {
            auto &row = cache.rows.emplace_back();
            row.J = {I.row[row_idx], -rA_skew.row[row_idx],
                    -I.row[row_idx], rB_skew.row[row_idx]};
            row.lower_limit = -EDYN_SCALAR_MAX;
            row.upper_limit = EDYN_SCALAR_MAX;

            row.inv_mA = inv_mA; row.inv_IA = inv_IA;
            row.inv_mB = inv_mB; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dwA = &dwA;
            row.dvB = &dvB; row.dwB = &dwB;
            row.impulse = imp.values[row_idx];

            prepare_row(row, {}, linvelA, angvelA, linvelB, angvelB);
            warm_start(row);
        }

        auto axisA = rotate(ornA, con.axis[0]);
        vector3 p, q;
        plane_space(axisA, p, q);

        {
            auto &row = cache.rows.emplace_back();
            row.J = {vector3_zero, p, vector3_zero, -p};
            row.lower_limit = -EDYN_SCALAR_MAX;
            row.upper_limit = EDYN_SCALAR_MAX;

            row.inv_mA = inv_mA; row.inv_IA = inv_IA;
            row.inv_mB = inv_mB; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dwA = &dwA;
            row.dvB = &dvB; row.dwB = &dwB;
            row.impulse = imp.values[row_idx++];

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
            row.impulse = imp.values[row_idx++];

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
    auto com_view = registry.view<center_of_mass>();
    auto linear_error = scalar(0);
    auto angular_error = scalar(0);

    con_view.each([&] (hinge_constraint &con) {
        auto [posA, ornA, inv_mA, inv_IA] =
            body_view.get<position, orientation, mass_inv, inertia_world_inv>(con.body[0]);
        auto [posB, ornB, inv_mB, inv_IB] =
            body_view.get<position, orientation, mass_inv, inertia_world_inv>(con.body[1]);

        auto originA = static_cast<vector3>(posA);
        auto originB = static_cast<vector3>(posB);

        if (com_view.contains(con.body[0])) {
            auto &com = com_view.get(con.body[0]);
            originA = to_world_space(-com, posA, ornA);
        }

        if (com_view.contains(con.body[1])) {
            auto &com = com_view.get(con.body[1]);
            originB = to_world_space(-com, posB, ornB);
        }

        auto axisA = rotate(ornA, con.axis[0]);
        auto axisB = rotate(ornB, con.axis[1]);

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
            auto J_invM_JT = dot(J[0], J[0]) * inv_mA +
                             dot(inv_IA * J[1], J[1]) +
                             dot(J[2], J[2]) * inv_mB +
                             dot(inv_IB * J[3], J[3]);
            auto eff_mass = scalar(1) / J_invM_JT;
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
