#include <entt/entt.hpp>
#include "edyn/constraints/point_constraint.hpp"
#include "edyn/comp/constraint.hpp"
#include "edyn/comp/constraint_row.hpp"
#include "edyn/comp/position.hpp"

namespace edyn {

void point_constraint::prepare(constraint *con, entt::registry &registry, scalar dt) {
    auto posA = registry.get<position>(con->entity[0]);
    auto posB = registry.get<position>(con->entity[1]);

    auto rA = pivot[0];
    auto rB = pivot[1];

    /* if (auto q = registry.try_get<orientation>(con->entity[0])) {
        rA = rotate(*q, rA);
    } */

    auto rA_skew = skew(rA);
    auto rB_skew = skew(rB);
    constexpr auto I = matrix3x3::identity;

    for (size_t i = 0; i < 3; ++i) {
        auto &row = registry.get<constraint_row>(con->row[i]);
        row.J = {I[i], -rA_skew[i], -I[i], rB_skew[i]};
        row.error = (posA[i] + rA[i] - posB[i] - rB[i]) / dt;
        row.lower_limit = -large_scalar;
        row.upper_limit = large_scalar;
    }
}

}