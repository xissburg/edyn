#include "edyn/constraints/spin_constraint.hpp"
#include "edyn/comp/constraint.hpp"
#include "edyn/comp/constraint_row.hpp"
#include "edyn/comp/relation.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/spin.hpp"
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
}

void spin_constraint::prepare(entt::entity, constraint &con, const relation &rel, entt::registry &registry, scalar dt) {
    auto &qA = registry.get<const orientation>(rel.entity[0]);
    auto &qB = registry.get<const orientation>(rel.entity[1]);
    auto &sA = registry.get<const spin>(rel.entity[0]);
    auto &sB = registry.get<const spin>(rel.entity[1]);

    auto axisA = rotate(qA, vector3_x);
    auto axisB = rotate(qB, vector3_x);

    auto impulse = m_max_torque * dt;

    auto &row = registry.get<constraint_row>(con.row[0]);
    row.J = {vector3_zero, -axisA, vector3_zero, axisB};
    row.error = 0;
    row.lower_limit = -impulse;
    row.upper_limit = impulse;
}

}