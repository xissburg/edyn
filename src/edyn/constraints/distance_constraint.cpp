#include "edyn/constraints/distance_constraint.hpp"
#include "edyn/comp/constraint.hpp"
#include "edyn/comp/constraint_row.hpp"
#include "edyn/comp/relation.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include <entt/entt.hpp>

namespace edyn {

void distance_constraint::init(constraint &con, const relation &rel, entt::registry &registry) {
    con.num_rows = 1;
    con.row[0] = registry.create();
    auto &row = registry.assign<constraint_row>(con.row[0]);
    row.entity = rel.entity;
}

void distance_constraint::prepare(constraint &con, const relation &rel, entt::registry &registry, scalar dt) {
    auto &posA = registry.get<const position>(rel.entity[0]);
    auto &posB = registry.get<const position>(rel.entity[1]);

    auto rA = pivot[0];
    auto rB = pivot[1];

    auto &qA = registry.get<const orientation>(rel.entity[0]);
    rA = rotate(qA, rA);

    auto &qB = registry.get<const orientation>(rel.entity[1]);
    rB = rotate(qB, rB);

    auto d = posA + rA - posB - rB;
    auto l2 = std::max(length2(d), EDYN_EPSILON);
    auto l = std::max(std::sqrt(l2), EDYN_EPSILON);
    auto dn = d / l;

    auto &linvelA = registry.get<const linvel>(rel.entity[0]);
    auto &angvelA = registry.get<const angvel>(rel.entity[0]);
    auto &linvelB = registry.get<const linvel>(rel.entity[1]);
    auto &angvelB = registry.get<const angvel>(rel.entity[1]);
    auto relvel = dot(linvelA + cross(angvelA, rA) - linvelB - cross(angvelB, rB), dn);

    auto force = stiffness * l + damping * std::abs(relvel);
    auto impulse = force * dt;

    auto &row = registry.get<constraint_row>(con.row[0]);
    row.J = {d, cross(rA, d), -d, -cross(rB, d)};
    row.error = scalar(0.5) * (l2 - distance * distance) / dt;
    row.lower_limit = -impulse;
    row.upper_limit = impulse;
}

}