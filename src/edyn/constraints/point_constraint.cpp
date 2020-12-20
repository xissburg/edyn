#include "edyn/constraints/point_constraint.hpp"
#include "edyn/comp/constraint.hpp"
#include "edyn/comp/constraint_row.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/util/constraint.hpp"
#include <entt/entt.hpp>

namespace edyn {

void point_constraint::init(entt::entity entity, constraint &con, entt::registry &registry) {
    for (size_t i = 0; i < 3; ++i) {
        add_constraint_row(entity, con, registry, 100);
    }
}

void point_constraint::prepare(entt::entity, constraint &con, entt::registry &registry, scalar dt) {
    auto &posA = registry.get<position>(con.body[0]);
    auto &posB = registry.get<position>(con.body[1]);

    auto rA = pivot[0];
    auto rB = pivot[1];

    auto &qA = registry.get<orientation>(con.body[0]);
    rA = rotate(qA, rA);

    auto &qB = registry.get<orientation>(con.body[1]);
    rB = rotate(qB, rB);

    auto rA_skew = skew(rA);
    auto rB_skew = skew(rB);
    constexpr auto I = matrix3x3_identity;

    for (size_t i = 0; i < 3; ++i) {
        auto &row = registry.get<constraint_row>(con.row[i]);
        row.J = {I.row[i], -rA_skew.row[i], -I.row[i], rB_skew.row[i]};
        row.error = (posA[i] + rA[i] - posB[i] - rB[i]) / dt;
        row.lower_limit = -EDYN_SCALAR_MAX;
        row.upper_limit = EDYN_SCALAR_MAX;
    }
}

}