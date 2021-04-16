#include "edyn/constraints/hinge_constraint.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/dynamics/row_cache.hpp"
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

void prepare_hinge_constraints(entt::registry &registry, row_cache &cache, scalar dt) {
    auto body_view = registry.view<position, orientation>();
    auto con_view = registry.view<hinge_constraint>();
    con_view.each([&] (hinge_constraint &con) {
        auto [posA, ornA] = body_view.get<position, orientation>(con.body[0]);
        auto [posB, ornB] = body_view.get<position, orientation>(con.body[1]);

        const auto rA = rotate(ornA, con.pivot[0]);
        const auto rB = rotate(ornB, con.pivot[1]);

        const auto rA_skew = skew_matrix(rA);
        const auto rB_skew = skew_matrix(rB);
        constexpr auto I = matrix3x3_identity;

        for (size_t i = 0; i < 3; ++i) {
            auto [row, data] = cache.make_row();
            data.J = {I.row[i], -rA_skew.row[i], -I.row[i], rB_skew.row[i]};
            data.lower_limit = -EDYN_SCALAR_MAX;
            data.upper_limit = EDYN_SCALAR_MAX;
            row.error = (posA[i] + rA[i] - posB[i] - rB[i]) / dt;
        }

        const auto n = rotate(ornA, con.frame[0].column(2));
        const auto p = rotate(ornA, con.frame[0].column(0));
        const auto q = rotate(ornA, con.frame[0].column(1));

        const auto m = rotate(ornB, con.frame[1].column(2));
        const auto u = cross(n, m);

        {
            auto [row, data] = cache.make_row();
            data.J = {vector3_zero, p, vector3_zero, -p};
            data.lower_limit = -EDYN_SCALAR_MAX;
            data.upper_limit = EDYN_SCALAR_MAX;
            row.error = dot(u, p) / dt;
        }

        {
            auto [row, data] = cache.make_row();
            data.J = {vector3_zero, q, vector3_zero, -q};
            data.lower_limit = -EDYN_SCALAR_MAX;
            data.upper_limit = EDYN_SCALAR_MAX;
            row.error = dot(u, q) / dt;
        }
    });
}

}