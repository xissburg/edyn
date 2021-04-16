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
#include "edyn/dynamics/row_cache.hpp"
#include "edyn/util/constraint_util.hpp"
#include <entt/entt.hpp>

namespace edyn {

void prepare_generic_constraints(entt::registry &registry, row_cache &cache, scalar dt) {
    auto body_view = registry.view<position, orientation, 
                                   linvel, angvel, 
                                   mass_inv, inertia_world_inv, 
                                   delta_linvel, delta_angvel>();
    auto con_view = registry.view<generic_constraint, constraint_impulse>();

    con_view.each([&] (generic_constraint &con, constraint_impulse &imp) {
        auto [posA, ornA, linvelA, angvelA, inv_mA, inv_IA, dvA, dwA] = 
            body_view.get<position, orientation, linvel, angvel, mass_inv, inertia_world_inv, delta_linvel, delta_angvel>(con.body[0]);
        auto [posB, ornB, linvelB, angvelB, inv_mB, inv_IB, dvB, dwB] = 
            body_view.get<position, orientation, linvel, angvel, mass_inv, inertia_world_inv, delta_linvel, delta_angvel>(con.body[1]);

        auto rA = rotate(ornA, con.pivot[0]);
        auto rB = rotate(ornB, con.pivot[1]);

        auto rA_skew = skew_matrix(rA);
        auto rB_skew = skew_matrix(rB);
        const auto d = posA + rA - posB - rB;
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