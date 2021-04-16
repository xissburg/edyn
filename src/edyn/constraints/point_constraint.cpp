#include "edyn/constraints/point_constraint.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/dynamics/row_cache.hpp"
#include <entt/entt.hpp>

namespace edyn {

void prepare_point_constraints(entt::registry &registry, row_cache &cache, scalar dt) {
    auto body_view = registry.view<position, orientation>();
    auto con_view = registry.view<point_constraint>();
    con_view.each([&] (point_constraint &con) {
        auto [posA, ornA] = body_view.get<position, orientation>(con.body[0]);
        auto [posB, ornB] = body_view.get<position, orientation>(con.body[1]);

        auto rA = con.pivot[0];
        auto rB = con.pivot[1];

        rA = rotate(ornA, rA);
        rB = rotate(ornB, rB);

        auto rA_skew = skew_matrix(rA);
        auto rB_skew = skew_matrix(rB);
        constexpr auto I = matrix3x3_identity;

        for (size_t i = 0; i < 3; ++i) {
            auto [row, data] = cache.make_row();
            data.J = {I.row[i], -rA_skew.row[i], -I.row[i], rB_skew.row[i]};
            data.lower_limit = -EDYN_SCALAR_MAX;
            data.upper_limit = EDYN_SCALAR_MAX;
            row.error = (posA[i] + rA[i] - posB[i] - rB[i]) / dt;
        }
    });
}

}