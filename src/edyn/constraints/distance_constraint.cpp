#include "edyn/constraints/distance_constraint.hpp"
#include "edyn/comp/constraint.hpp"
#include "edyn/comp/constraint_row.hpp"
#include "edyn/comp/relation.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/util/constraint.hpp"
#include <entt/entt.hpp>

namespace edyn {

void distance_constraint::init(entt::entity, constraint &con, const relation &rel, entt::registry &registry) {
    con.num_rows = 1;
    con.row[0] = registry.create();
    auto &row = registry.assign<constraint_row>(con.row[0]);
    row.entity = rel.entity;
    row.priority = 400;
}

void distance_constraint::prepare(entt::entity, constraint &con, const relation &rel, entt::registry &registry, scalar dt) {
    auto &posA = registry.get<const position>(rel.entity[0]);
    auto &ornA = registry.get<const orientation>(rel.entity[0]);
    auto &posB = registry.get<const position>(rel.entity[1]);
    auto &ornB = registry.get<const orientation>(rel.entity[1]);

    auto rA = rotate(ornA, pivot[0]);
    auto rB = rotate(ornB, pivot[1]);

    auto d = posA + rA - posB - rB;
    auto l2 = length2(d);
    
    if (l2 <= EDYN_EPSILON) {
        d = vector3_x;
    }

    auto &row = registry.get<constraint_row>(con.row[0]);
    row.J = {d, cross(rA, d), -d, -cross(rB, d)};
    row.error = scalar(0.5) * (l2 - distance * distance) / dt;
    row.lower_limit = -large_scalar;
    row.upper_limit =  large_scalar;
}

}