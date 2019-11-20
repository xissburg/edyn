#include <entt/entt.hpp>
#include "edyn/constraints/point_constraint.hpp"
#include "edyn/comp/constraint.hpp"
#include "edyn/comp/constraint_row.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/math/matrix3x3.hpp"

namespace edyn {

void point_constraint::prepare(constraint *con, entt::registry &registry, scalar dt) {
    auto &posA = registry.get<const position>(con->entity[0]);
    auto &posB = registry.get<const position>(con->entity[1]);

    auto rA = pivot[0];
    auto rB = pivot[1];

    /* if (auto q = registry.try_get<orientation>(con->entity[0])) {
        rA = rotate(*q, rA);
    } */

    auto rA_skew = skew(rA);
    auto rB_skew = skew(rB);
    constexpr auto I = matrix3x3_identity;

    for (size_t i = 0; i < 3; ++i) {
        auto &row = registry.get<constraint_row>(con->row[i]);
        row.J = {I.row[i], -rA_skew.row[i], -I.row[i], rB_skew.row[i]};
        row.error = (posB[i] + rB[i] - posA[i] - rA[i]) / dt;
        row.lower_limit = -EDYN_SCALAR_MAX;
        row.upper_limit = EDYN_SCALAR_MAX;
    }
}

}