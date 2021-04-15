#include "edyn/constraints/generic_constraint.hpp"
#include "edyn/comp/constraint.hpp"
#include "edyn/comp/constraint_row.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/util/constraint_util.hpp"
#include "edyn/dynamics/row_cache.hpp"
#include <entt/entt.hpp>

namespace edyn {

void generic_constraint::prepare(entt::entity entity, const constraint &con, 
                                 entt::registry &registry, row_cache &cache, 
                                 scalar dt) {
    auto &posA = registry.get<position>(con.body[0]);
    auto &posB = registry.get<position>(con.body[1]);

    auto &ornA = registry.get<orientation>(con.body[0]);
    auto rA = rotate(ornA, pivot[0]);

    auto &ornB = registry.get<orientation>(con.body[1]);
    auto rB = rotate(ornB, pivot[1]);

    auto rA_skew = skew_matrix(rA);
    auto rB_skew = skew_matrix(rB);
    const auto d = posA + rA - posB - rB;
    constexpr auto I = matrix3x3_identity;

    // Linear.
    for (size_t i = 0; i < 3; ++i) {
        auto [row, data] = cache.make_row();
        auto p = rotate(ornA, I.row[i]);
        data.J = {p, rA_skew.row[i], -p, -rB_skew.row[i]};
        data.lower_limit = -large_scalar;
        data.upper_limit = large_scalar;
        row.error = dot(p, d) / dt;
    }

    // Angular.
    for (size_t i = 0; i < 3; ++i) {
        auto [row, data] = cache.make_row();
        auto axis = rotate(ornA, I.row[i]);
        auto n = rotate(ornA, I.row[(i+1)%3]);
        auto m = rotate(ornB, I.row[(i+2)%3]);
        auto error = dot(n, m);

        data.J = {vector3_zero, axis, vector3_zero, -axis};
        data.lower_limit = -large_scalar;
        data.upper_limit = large_scalar;
        row.error = error / dt;
    }
}

}