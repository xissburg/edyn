#include "edyn/constraints/spin_constraint.hpp"
#include "edyn/comp/constraint.hpp"
#include "edyn/comp/constraint_row.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/spin.hpp"
#include "edyn/util/constraint_util.hpp"
#include <entt/entt.hpp>

namespace edyn {

void spin_constraint::init(entt::entity entity, constraint &con, entt::registry &registry) {
    auto row_entity = add_constraint_row(entity, con, registry, 400);
    auto &row = registry.get<constraint_row>(row_entity);
    row.use_spin[0] = true;
    row.use_spin[1] = true;
}

void spin_constraint::prepare(entt::entity, constraint &con, entt::registry &registry, scalar dt) {
    auto &qA = registry.get<orientation>(con.body[0]);
    auto &qB = registry.get<orientation>(con.body[1]);

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