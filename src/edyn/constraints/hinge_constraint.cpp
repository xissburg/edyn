#include "edyn/constraints/hinge_constraint.hpp"
#include "edyn/constraints/constraint_impulse.hpp"
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
#include "edyn/math/geom.hpp"
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

            auto options = constraint_row_options{};
            options.error = (pivotA[row_idx] - pivotB[row_idx]) / dt;

            prepare_row(row, options, linvelA, linvelB, angvelA, angvelB);
            warm_start(row);
        }

        const auto n = rotate(ornA, con.axis[0]);
        const auto m = rotate(ornB, con.axis[1]);
        const auto u = cross(n, m);

        vector3 p, q;
        plane_space(n, p, q);

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

            auto options = constraint_row_options{};
            options.error = -dot(u, p) / dt;

            prepare_row(row, options, linvelA, linvelB, angvelA, angvelB);
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

            auto options = constraint_row_options{};
            options.error = -dot(u, q) / dt;

            prepare_row(row, options, linvelA, linvelB, angvelA, angvelB);
            warm_start(row);
        }

        cache.con_num_rows.push_back(row_idx);
    });
}

}
