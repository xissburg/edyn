#include "edyn/constraints/hinge_constraint.hpp"
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

void hinge_constraint::set_axis(const quaternion &ornA,
                                const vector3 &axisA, const vector3 &axisB) {
    auto frameA_x = rotate(ornA, vector3_x);
    vector3 frameA_y;
    auto proj = dot(axisA, frameA_x);

    if (proj >= 1 - EDYN_EPSILON) {
        frameA_x = -rotate(ornA, vector3_z);
        frameA_y = rotate(ornA, vector3_y);
    } else if (proj <= -1 + EDYN_EPSILON) {
        frameA_x = rotate(ornA, vector3_z);
        frameA_y = rotate(ornA, vector3_y);
    } else {
        frameA_y = cross(axisA, frameA_x);
        frameA_x = cross(frameA_y, axisA);
    }

    frame[0] = matrix3x3_columns(frameA_x, frameA_y, axisA);

    auto arc = shortest_arc(axisA, axisB);
    auto frameB_x = rotate(arc, frameA_x);
    auto frameB_y = cross(axisB, frameB_x);

    frame[1] = matrix3x3_columns(frameB_x, frameB_y, axisB);
}

template<>
void prepare_constraints<hinge_constraint>(entt::registry &registry, row_cache &cache, scalar dt) {
    auto body_view = registry.view<position, orientation, 
                                   linvel, angvel, 
                                   mass_inv, inertia_world_inv, 
                                   delta_linvel, delta_angvel>();
    auto con_view = registry.view<hinge_constraint, constraint_impulse>();

    con_view.each([&] (hinge_constraint &con, constraint_impulse &imp) {
        auto [posA, ornA, linvelA, angvelA, inv_mA, inv_IA, dvA, dwA] = 
            body_view.get<position, orientation, linvel, angvel, mass_inv, inertia_world_inv, delta_linvel, delta_angvel>(con.body[0]);
        auto [posB, ornB, linvelB, angvelB, inv_mB, inv_IB, dvB, dwB] = 
            body_view.get<position, orientation, linvel, angvel, mass_inv, inertia_world_inv, delta_linvel, delta_angvel>(con.body[1]);

        const auto rA = rotate(ornA, con.pivot[0]);
        const auto rB = rotate(ornB, con.pivot[1]);

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
            options.error = (posA[row_idx] + rA[row_idx] - posB[row_idx] - rB[row_idx]) / dt;

            prepare_row(row, options, linvelA, linvelB, angvelA, angvelB);
            warm_start(row);
        }

        const auto n = rotate(ornA, con.frame[0].column(2));
        const auto p = rotate(ornA, con.frame[0].column(0));
        const auto q = rotate(ornA, con.frame[0].column(1));

        const auto m = rotate(ornB, con.frame[1].column(2));
        const auto u = cross(n, m);

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
            options.error = dot(u, p) / dt;

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
            options.error = dot(u, q) / dt;

            prepare_row(row, options, linvelA, linvelB, angvelA, angvelB);
            warm_start(row);
        }

        cache.con_num_rows.push_back(row_idx);
    });
}

}