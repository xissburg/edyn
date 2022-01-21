#include "edyn/constraints/point_constraint.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/math/transform.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
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

namespace edyn {

template<>
void prepare_constraints<point_constraint>(entt::registry &registry, row_cache &cache, scalar dt) {
    auto body_view = registry.view<position, orientation,
                                   linvel, angvel,
                                   mass_inv, inertia_world_inv,
                                   delta_linvel, delta_angvel>();
    auto con_view = registry.view<point_constraint>(entt::exclude_t<disabled_tag>{});
    auto origin_view = registry.view<origin>();

    con_view.each([&] (point_constraint &con) {
        auto [posA, ornA, linvelA, angvelA, inv_mA, inv_IA, dvA, dwA] = body_view.get(con.body[0]);
        auto [posB, ornB, linvelB, angvelB, inv_mB, inv_IB, dvB, dwB] = body_view.get(con.body[1]);

        auto originA = origin_view.contains(con.body[0]) ? origin_view.get<origin>(con.body[0]) : static_cast<vector3>(posA);
        auto originB = origin_view.contains(con.body[1]) ? origin_view.get<origin>(con.body[1]) : static_cast<vector3>(posB);

        auto pivotA = to_world_space(con.pivot[0], originA, ornA);
        auto pivotB = to_world_space(con.pivot[1], originB, ornB);
        auto rA = pivotA - posA;
        auto rB = pivotB - posB;

        auto rA_skew = skew_matrix(rA);
        auto rB_skew = skew_matrix(rB);
        constexpr auto I = matrix3x3_identity;
        auto num_rows = size_t{3};

        for (size_t i = 0; i < num_rows; ++i) {
            auto &row = cache.rows.emplace_back();
            row.J = {I.row[i], -rA_skew.row[i], -I.row[i], rB_skew.row[i]};
            row.lower_limit = -EDYN_SCALAR_MAX;
            row.upper_limit = EDYN_SCALAR_MAX;

            row.inv_mA = inv_mA;  row.inv_IA = inv_IA;
            row.inv_mB = inv_mB; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dwA = &dwA;
            row.dvB = &dvB; row.dwB = &dwB;
            row.impulse = con.impulse[i];

            auto options = constraint_row_options{};
            options.error = (pivotA[i] - pivotB[i]) / dt;

            prepare_row(row, options, linvelA, angvelA, linvelB, angvelB);
            warm_start(row);
        }

        if (con.friction_torque > 0) {
            auto spin_axis = angvelA - angvelB;

            if (try_normalize(spin_axis)) {
                auto &row = cache.rows.emplace_back();
                row.J = {vector3_zero, spin_axis, vector3_zero, -spin_axis};
                row.inv_mA = inv_mA;  row.inv_IA = inv_IA;
                row.inv_mB = inv_mB; row.inv_IB = inv_IB;
                row.dvA = &dvA; row.dwA = &dwA;
                row.dvB = &dvB; row.dwB = &dwB;
                row.impulse = con.impulse[++num_rows];

                auto friction_impulse = con.friction_torque * dt;
                row.lower_limit = -friction_impulse;
                row.upper_limit = friction_impulse;

                prepare_row(row, {}, linvelA, angvelA, linvelB, angvelB);
                warm_start(row);
            }
        }

        cache.con_num_rows.push_back(num_rows);
    });
}

}
