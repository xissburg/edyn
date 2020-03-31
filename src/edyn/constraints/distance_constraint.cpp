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

void distance_constraint::init(entt::entity, constraint &con, const relation &rel, entt::registry &registry) {
    con.num_rows = 1;
    con.row[0] = registry.create();
    auto &row = registry.assign<constraint_row>(con.row[0]);
    row.entity = rel.entity;
    row.priority = 400;
}

void distance_constraint::prepare(entt::entity, constraint &con, const relation &rel, entt::registry &registry, scalar dt) {
    auto &posA = registry.get<const position>(rel.entity[0]);
    auto &posB = registry.get<const position>(rel.entity[1]);
    auto &ornA = registry.get<const orientation>(rel.entity[0]);
    auto &ornB = registry.get<const orientation>(rel.entity[1]);

    auto &linvelA = registry.get<const linvel>(rel.entity[0]);
    auto &angvelA = registry.get<const angvel>(rel.entity[0]);
    auto &linvelB = registry.get<const linvel>(rel.entity[1]);
    auto &angvelB = registry.get<const angvel>(rel.entity[1]);

    auto rA = rotate(ornA, pivot[0]);
    auto rB = rotate(ornB, pivot[1]);

    auto d = posA + rA - posB - rB;
    auto l2 = length2(d);
    auto l = std::sqrt(l2);
    vector3 dn;
    
    if (l2 <= EDYN_EPSILON) {
        d = dn = vector3_x;
    } else {
        dn = d / l;
    }

    // Use a strategy similar to what's done in Bullet's `btGeneric6DofSpring2Constraint`.
    // The row's error is set to a large number with the opposite sign of the impulse,
    // thus the limit will determine the impulse to be applied.
    
    auto error = l - distance;
    auto spring_force = -stiffness * error;
    auto spring_impulse = spring_force * dt;

    auto relvel = dot(linvelA + cross(angvelA, rA) - linvelB - cross(angvelB, rB), dn);
    auto damping_force = -damping * relvel;
    auto damping_impulse = damping_force * dt;

    auto impulse = spring_impulse + damping_impulse;
    auto min_impulse = std::min(scalar(0), std::min(impulse, damping_impulse));
    auto max_impulse = std::max(scalar(0), std::max(impulse, damping_impulse));

    auto &row = registry.get<constraint_row>(con.row[0]);
    row.J = {dn, cross(rA, dn), -dn, -cross(rB, dn)};
    row.error = impulse > 0 ? -large_scalar : large_scalar;
    row.lower_limit = min_impulse;
    row.upper_limit = max_impulse;

}

}