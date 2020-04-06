#include "edyn/constraints/soft_distance_constraint.hpp"
#include "edyn/comp/constraint.hpp"
#include "edyn/comp/constraint_row.hpp"
#include "edyn/comp/relation.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/delta_linvel.hpp"
#include "edyn/comp/delta_angvel.hpp"
#include "edyn/util/constraint.hpp"
#include <entt/entt.hpp>

namespace edyn {

void soft_distance_constraint::init(entt::entity, constraint &con, 
                                    const relation &rel, entt::registry &registry) {
    con.num_rows = 2;
    for (size_t i = 0; i < con.num_rows; ++i) {
        con.row[i] = registry.create();
        auto &row = registry.assign<constraint_row>(con.row[i]);
        row.entity = rel.entity;
        row.priority = 400;
    }
}

void soft_distance_constraint::prepare(entt::entity, constraint &con, 
                                       const relation &rel, entt::registry &registry, scalar dt) {
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
    
    if (l2 > EDYN_EPSILON) {
        dn = d / l;
    } else {
        d = dn = vector3_x;
    }

    {
        // Spring row. By setting the error to +/- `large_scalar`, it will
        // always apply the impulse set in the limits.
        auto error = distance - l;
        auto spring_force = stiffness * error;
        auto spring_impulse = spring_force * dt;

        auto &row = registry.get<constraint_row>(con.row[0]);
        row.J = {dn, cross(rA, dn), -dn, -cross(rB, dn)};
        row.error = spring_impulse > 0 ? -large_scalar : large_scalar;
        row.lower_limit = std::min(spring_impulse, scalar(0));
        row.upper_limit = std::max(scalar(0), spring_impulse);
    }

    {
        // Damping row. It functions like friction where the force is
        // proportional to the relative speed.
        auto &row = registry.get<constraint_row>(con.row[1]);
        row.J = {dn, cross(rA, dn), -dn, -cross(rB, dn)};
        row.error = 0;

        auto &linvelA = registry.get<linvel>(rel.entity[0]);
        auto &angvelA = registry.get<angvel>(rel.entity[0]);
        auto &linvelB = registry.get<linvel>(rel.entity[1]);
        auto &angvelB = registry.get<angvel>(rel.entity[1]);

        auto relspd = dot(row.J[0], linvelA) + 
                      dot(row.J[1], angvelA) +
                      dot(row.J[2], linvelB) +
                      dot(row.J[3], angvelB);
        auto damping_force = damping * relspd;
        auto damping_impulse = damping_force * dt;
        auto impulse = std::abs(damping_impulse);

        row.lower_limit = -impulse;
        row.upper_limit =  impulse;

        m_relspd = relspd;
    }
}

void soft_distance_constraint::iteration(entt::entity entity, constraint &con, 
                                         const relation &rel, entt::registry &registry, scalar dt) {
    // Adjust damping row limits to account for velocity changes during iterations.
    auto &dvA = registry.get<delta_linvel>(rel.entity[0]);
    auto &dwA = registry.get<delta_angvel>(rel.entity[0]);
    auto &dvB = registry.get<delta_linvel>(rel.entity[1]);
    auto &dwB = registry.get<delta_angvel>(rel.entity[1]);

    auto &row = registry.get<constraint_row>(con.row[1]);
    auto delta_relspd = dot(row.J[0], dvA) + 
                        dot(row.J[1], dwA) +
                        dot(row.J[2], dvB) +
                        dot(row.J[3], dwB);

    auto relspd = m_relspd + delta_relspd;

    auto damping_force = damping * relspd;
    auto damping_impulse = damping_force * dt;
    auto impulse = std::abs(damping_impulse);

    row.lower_limit = -impulse;
    row.upper_limit =  impulse;
}

}