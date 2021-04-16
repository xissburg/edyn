#include "edyn/constraints/point_constraint.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/constraints/constraint_impulse.hpp"
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

void prepare_point_constraints(entt::registry &registry, row_cache &cache, scalar dt) {
    auto body_view = registry.view<position, orientation, 
                                   linvel, angvel, 
                                   mass_inv, inertia_world_inv, 
                                   delta_linvel, delta_angvel>();
    auto con_view = registry.view<point_constraint, constraint_impulse>();

    con_view.each([&] (point_constraint &con, constraint_impulse& imp) {
        auto [posA, ornA, linvelA, angvelA, inv_mA, inv_IA, dvA, dwA] = 
            body_view.get<position, orientation, linvel, angvel, mass_inv, inertia_world_inv, delta_linvel, delta_angvel>(con.body[0]);
        auto [posB, ornB, linvelB, angvelB, inv_mB, inv_IB, dvB, dwB] = 
            body_view.get<position, orientation, linvel, angvel, mass_inv, inertia_world_inv, delta_linvel, delta_angvel>(con.body[1]);

        auto rA = con.pivot[0];
        auto rB = con.pivot[1];

        rA = rotate(ornA, rA);
        rB = rotate(ornB, rB);

        auto rA_skew = skew_matrix(rA);
        auto rB_skew = skew_matrix(rB);
        constexpr auto I = matrix3x3_identity;

        for (size_t i = 0; i < 3; ++i) {
            auto &row = cache.con_rows.emplace_back();
            row.J = {I.row[i], -rA_skew.row[i], -I.row[i], rB_skew.row[i]};
            row.lower_limit = -EDYN_SCALAR_MAX;
            row.upper_limit = EDYN_SCALAR_MAX;

            row.inv_mA = inv_mA; row.inv_mB = inv_mB;
            row.inv_IA = inv_IA; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dvB = &dvB;
            row.dwA = &dwA; row.dwB = &dwB;
            row.impulse = imp.values[i];

            auto options = constraint_row_options{};
            options.error = (posA[i] + rA[i] - posB[i] - rB[i]) / dt;

            prepare_row(row, options, linvelA, linvelB, angvelA, angvelB);
            warm_start(row);
        }

        cache.con_num_rows.push_back(3);
    });
}

}