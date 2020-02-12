#include "edyn/constraints/spin_constraint.hpp"
#include "edyn/comp/constraint.hpp"
#include "edyn/comp/constraint_row.hpp"
#include "edyn/comp/relation.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/spin.hpp"
#include "edyn/math/math.hpp"
#include <entt/entt.hpp>

namespace edyn {

void spin_constraint::init(entt::entity, constraint &con, const relation &rel, entt::registry &registry) {
    con.num_rows = 1;
    con.row[0] = registry.create();
    auto &row = registry.assign<constraint_row>(con.row[0]);
    row.entity = rel.entity;
    row.priority = 400;
    row.use_spin[0] = true;
    row.use_spin[1] = true;
    
    m_offset = get_error(rel, registry);
}

void spin_constraint::prepare(entt::entity, constraint &con, const relation &rel, entt::registry &registry, scalar dt) {
    auto &qA = registry.get<const orientation>(rel.entity[0]);
    auto &qB = registry.get<const orientation>(rel.entity[1]);
    auto &sA = registry.get<const spin>(rel.entity[0]);
    auto &sB = registry.get<const spin>(rel.entity[1]);

    auto axisA = rotate(qA, vector3_x);
    auto axisB = rotate(qB, vector3_x);

    auto error = get_error(rel, registry) - m_offset;
    auto relvel = sA - sB * m_ratio;
    auto force = error * m_stiffness + relvel * m_damping;
    auto impulse = std::abs(force) * dt;

    auto &row = registry.get<constraint_row>(con.row[0]);
    row.J = {vector3_zero, -axisA, vector3_zero, axisB * m_ratio};
    row.error = error / dt;
    row.lower_limit = -impulse;
    row.upper_limit = impulse;
}

void spin_constraint::set_ratio(scalar ratio, const relation &rel, entt::registry &registry) {
    m_ratio = ratio;
    m_offset = get_error(rel, registry);
}

scalar spin_constraint::get_error(const relation &rel, entt::registry &registry) const {
    auto &sA = registry.get<spin_angle>(rel.entity[0]);
    auto &sB = registry.get<spin_angle>(rel.entity[1]);
    return (sA.count - sB.count * m_ratio) * pi2 + (sA + sB * m_ratio);
}

}