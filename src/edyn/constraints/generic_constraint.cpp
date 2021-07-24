#include "edyn/constraints/generic_constraint.hpp"
#include "edyn/constraints/constraint_row.hpp"
#include "edyn/constraints/constraint_impulse.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/math/matrix3x3.hpp"
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
void prepare_constraints<generic_constraint>(entt::registry &registry, row_cache &cache, scalar dt) {
    auto body_view = registry.view<position, orientation,
                                   linvel, angvel,
                                   mass_inv, inertia_world_inv,
                                   delta_linvel, delta_angvel>();
    auto con_view = registry.view<generic_constraint, constraint_impulse>();
    auto com_view = registry.view<center_of_mass>();

    con_view.each([&] (generic_constraint &con, constraint_impulse &imp) {
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

        auto rA_skew = skew_matrix(rA);
        auto rB_skew = skew_matrix(rB);
        auto d = pivotA - pivotB;
        constexpr auto I = matrix3x3_identity;

        // Linear.
        for (size_t i = 0; i < 3; ++i) {
            auto &row = cache.rows.emplace_back();
            auto p = rotate(ornA, I.row[i]);
            row.J = {p, rA_skew.row[i], -p, -rB_skew.row[i]};
            row.lower_limit = -large_scalar;
            row.upper_limit = large_scalar;

            row.inv_mA = inv_mA; row.inv_IA = inv_IA;
            row.inv_mB = inv_mB; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dwA = &dwA;
            row.dvB = &dvB; row.dwB = &dwB;
            row.impulse = imp.values[i];

            auto options = constraint_row_options{};
            options.error = dot(p, d) / dt;

            prepare_row(row, options, linvelA, linvelB, angvelA, angvelB);
            warm_start(row);
        }

        // Angular.
        for (size_t i = 0; i < 3; ++i) {
            auto &row = cache.rows.emplace_back();
            auto axis = rotate(ornA, I.row[i]);
            auto n = rotate(ornA, I.row[(i+1)%3]);
            auto m = rotate(ornB, I.row[(i+2)%3]);
            auto error = dot(n, m);

            row.J = {vector3_zero, axis, vector3_zero, -axis};
            row.lower_limit = -large_scalar;
            row.upper_limit = large_scalar;

            row.inv_mA = inv_mA; row.inv_IA = inv_IA;
            row.inv_mB = inv_mB; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dwA = &dwA;
            row.dvB = &dvB; row.dwB = &dwB;
            row.impulse = imp.values[3 + i];

            auto options = constraint_row_options{};
            options.error = error / dt;

            prepare_row(row, options, linvelA, linvelB, angvelA, angvelB);
            warm_start(row);
        }

        cache.con_num_rows.push_back(6);
    });
}

}
