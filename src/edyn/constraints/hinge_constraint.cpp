#include "edyn/constraints/hinge_constraint.hpp"
#include "edyn/comp/relation.hpp"
#include "edyn/comp/constraint.hpp"
#include "edyn/comp/constraint_row.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/math/matrix3x3.hpp"
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

void hinge_constraint::init(constraint &con, const relation &rel, entt::registry &registry) {
    con.num_rows = 5;

    for (size_t i = 0; i < con.num_rows; ++i) {
        auto e = registry.create();
        con.row[i] = e;
        auto &row = registry.assign<constraint_row>(e);
        row.entity = rel.entity;
    }
}

void hinge_constraint::prepare(constraint &con, const relation &rel, entt::registry &registry, scalar dt) {
    auto &posA = registry.get<const position>(rel.entity[0]);
    auto &posB = registry.get<const position>(rel.entity[1]);

    auto rA = pivot[0];
    auto rB = pivot[1];

    auto &ornA = registry.get<const orientation>(rel.entity[0]);
    rA = rotate(ornA, rA);

    auto &ornB = registry.get<const orientation>(rel.entity[1]);
    rB = rotate(ornB, rB);

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

    auto n = rotate(ornA, frame[0].column(2));
    auto p = rotate(ornA, frame[0].column(0));
    auto q = rotate(ornA, frame[0].column(1));

    auto m = rotate(ornB, frame[1].column(2));
    auto u = cross(n, m);

    {
        auto &row = registry.get<constraint_row>(con.row[3]);
        row.J = {vector3_zero, p, vector3_zero, -p};
        row.error = dot(u, p) / dt;
        row.lower_limit = -EDYN_SCALAR_MAX;
        row.upper_limit = EDYN_SCALAR_MAX;
    }

    {
        auto &row = registry.get<constraint_row>(con.row[4]);
        row.J = {vector3_zero, q, vector3_zero, -q};
        row.error = dot(u, q) / dt;
        row.lower_limit = -EDYN_SCALAR_MAX;
        row.upper_limit = EDYN_SCALAR_MAX;
    }
}

}