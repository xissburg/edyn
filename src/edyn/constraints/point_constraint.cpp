#include "edyn/constraints/point_constraint.hpp"
#include "edyn/comp/relation.hpp"
#include "edyn/comp/constraint.hpp"
#include "edyn/comp/constraint_row.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/math/matrix3x3.hpp"
#include <entt/entt.hpp>

namespace edyn {

void point_constraint::init(constraint &con, const relation &rel, entt::registry &registry) {
    con.num_rows = 3;

    for (size_t i = 0; i < 3; ++i) {
        auto e = registry.create();
        con.row[i] = e;
        auto &row = registry.assign<constraint_row>(e);
        row.entity = rel.entity;
    }
}

void point_constraint::prepare(constraint &con, const relation &rel, entt::registry &registry, scalar dt) {
    auto &posA = registry.get<const position>(rel.entity[0]);
    auto &posB = registry.get<const position>(rel.entity[1]);

    auto rA = pivot[0];
    auto rB = pivot[1];

    auto &qA = registry.get<const orientation>(rel.entity[0]);
    rA = rotate(qA, rA);

    auto &qB = registry.get<const orientation>(rel.entity[1]);
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

void point_constraint::before_solve(constraint &con, const relation &rel, entt::registry &registry, scalar dt) {

}

}