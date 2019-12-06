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
    con.num_rows = 2;

    for (size_t i = 0; i < con.num_rows; ++i) {
        con.row[i] = registry.create();
        auto &row = registry.assign<constraint_row>(con.row[i]);
        row.entity = rel.entity;
    }
}

void distance_constraint::prepare(constraint &con, const relation &rel, entt::registry &registry, scalar dt) {
    auto &posA = registry.get<const position>(rel.entity[0]);
    auto &posB = registry.get<const position>(rel.entity[1]);

    auto &qA = registry.get<const orientation>(rel.entity[0]);
    auto rA = rotate(qA, pivot[0]);

    auto &qB = registry.get<const orientation>(rel.entity[1]);
    auto rB = rotate(qB, pivot[1]);

    auto d = posA + rA - posB - rB;
    auto l2 = std::max(length2(d), EDYN_EPSILON);
    auto l = std::max(std::sqrt(l2), EDYN_EPSILON);
    auto dn = d / l;

    {
        auto error = l - distance;
        auto force = std::abs(stiffness * error);
        auto impulse = force * dt;

        auto &row = registry.get<constraint_row>(con.row[0]);
        row.J = {d, cross(rA, d), -d, -cross(rB, d)};
        row.error = scalar(0.5) * (l2 - distance * distance) / dt;
        row.lower_limit = -impulse;
        row.upper_limit = impulse;
    }

    {
        auto &linvelA = registry.get<const linvel>(rel.entity[0]);
        auto &angvelA = registry.get<const angvel>(rel.entity[0]);
        auto &linvelB = registry.get<const linvel>(rel.entity[1]);
        auto &angvelB = registry.get<const angvel>(rel.entity[1]);

        auto relvel = dot(linvelA + cross(angvelA, rA) - linvelB - cross(angvelB, rB), dn);
        auto force = std::abs(damping * relvel);
        auto impulse = force * dt;

        auto &row = registry.get<constraint_row>(con.row[1]);
        row.J = {d, cross(rA, d), -d, -cross(rB, d)};
        row.error = 0;
        row.lower_limit = -impulse;
        row.upper_limit = impulse;
    }

}

}