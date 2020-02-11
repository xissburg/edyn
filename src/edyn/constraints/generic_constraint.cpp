#include "edyn/constraints/generic_constraint.hpp"
#include "edyn/comp/relation.hpp"
#include "edyn/comp/constraint.hpp"
#include "edyn/comp/constraint_row.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/math/matrix3x3.hpp"
#include <entt/entt.hpp>

namespace edyn {

void generic_constraint::init(entt::entity, constraint &con, const relation &rel, entt::registry &registry) {
    con.num_rows = 6;

    for (size_t i = 0; i < con.num_rows; ++i) {
        auto e = registry.create();
        con.row[i] = e;
        auto &row = registry.assign<constraint_row>(e);
        row.entity = rel.entity;
        row.priority = 100;
    }
}

void generic_constraint::prepare(entt::entity, constraint &con, const relation &rel, entt::registry &registry, scalar dt) {
    auto &posA = registry.get<const position>(rel.entity[0]);
    auto &posB = registry.get<const position>(rel.entity[1]);

    auto &ornA = registry.get<const orientation>(rel.entity[0]);
    auto rA = rotate(ornA, pivot[0]);

    auto &ornB = registry.get<const orientation>(rel.entity[1]);
    auto rB = rotate(ornB, pivot[1]);

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

    for (size_t i = 0; i < 3; ++i) {
        auto p = rotate(ornA, I.row[i]);
        auto q = rotate(ornB, I.row[i]);
        auto &row = registry.get<constraint_row>(con.row[i + 3]);
        row.J = {vector3_zero, p, vector3_zero, -p};
        row.error = 0;//(dot(q, p) - 1) / dt;
        row.lower_limit = -EDYN_SCALAR_MAX;
        row.upper_limit = EDYN_SCALAR_MAX;
    }
}

}