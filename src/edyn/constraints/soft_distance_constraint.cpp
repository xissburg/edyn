#include "edyn/constraints/soft_distance_constraint.hpp"
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

void soft_distance_constraint::init(entt::entity, constraint &con, const relation &rel, entt::registry &registry) {
    con.num_rows = 1;
    con.row[0] = registry.create();
    auto &row = registry.assign<constraint_row>(con.row[0]);
    row.entity = rel.entity;
    row.priority = 400;
}

void soft_distance_constraint::prepare(entt::entity, constraint &con, const relation &rel, entt::registry &registry, scalar dt) {
    auto &posA = registry.get<const position>(rel.entity[0]);
    auto &ornA = registry.get<const orientation>(rel.entity[0]);
    auto &posB = registry.get<const position>(rel.entity[1]);
    auto &ornB = registry.get<const orientation>(rel.entity[1]);

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

    auto &row = registry.get<constraint_row>(con.row[0]);
    row.J = {dn, cross(rA, dn), -dn, -cross(rB, dn)};
    row.error = 0;
    row.lower_limit = -large_scalar;
    row.upper_limit =  large_scalar;

    // Spring force is applied as an impulse directly to the entities, immediately
    // affecting their velocities below. Damping is later applied as a constraint,
    // which acts to slow the entities down.
    auto &linvelA = registry.get<linvel>(rel.entity[0]);
    auto &angvelA = registry.get<angvel>(rel.entity[0]);
    auto &linvelB = registry.get<linvel>(rel.entity[1]);
    auto &angvelB = registry.get<angvel>(rel.entity[1]);

    auto &inv_mA = registry.get<mass_inv>(rel.entity[0]);
    auto &inv_IA = registry.get<inertia_world_inv>(rel.entity[0]);
    auto &inv_mB = registry.get<mass_inv>(rel.entity[1]);
    auto &inv_IB = registry.get<inertia_world_inv>(rel.entity[1]);

    auto error = distance - l;
    auto spring_impulse = stiffness * error * dt * dn;

    linvelA += spring_impulse * inv_mA;
    angvelA += inv_IA * cross(rA, spring_impulse);
    linvelB -= spring_impulse * inv_mB;
    angvelB -= inv_IB * cross(rB, spring_impulse);

    // Damping is applied using a negative restitution. The restitution scales
    // the relative velocity thus increasing (if greater than zero) or 
    // decreasing (if smaller than zero) the total applied impulse in the solver.
    // The maximum damping rate `k_d` that would have an effect (i.e. it would
    // bring the relative velocity to zero after solving) is the damping rate
    // that would generate an impulse `P` high enough to zero out the current
    // relative velocity `v`.
    // k_d * v * h = P
    // k_d = P / (v * h)
    // The impulse to solve this constraint if it was rigid (i.e. the impulse
    // necessary to zero out the relative velocity) is `K * v` where `K` is the 
    // effective mass. Thus:
    // k_d = K / h
    // The restitution will then be the ratio between this constraint's damping
    // rate and the maximum damping, capped to 1, flipped by subtracting it from
    // one, and negated.
    auto relvel = dot(linvelA + cross(angvelA, rA) - linvelB - cross(angvelB, rB), dn);
    auto damping_force = damping * relvel;
    auto damping_impulse = damping_force * dt;

    auto eff_mass = get_effective_mass(row, inv_mA, inv_IA, inv_mB, inv_IB);
    auto max_damping = eff_mass / dt;
    auto damping_ratio = std::min(damping / max_damping, scalar(1));
    row.restitution = -(1 - damping_ratio);
}

}