#include "edyn/constraints/spin_angle_constraint.hpp"
#include "edyn/comp/constraint.hpp"
#include "edyn/comp/constraint_row.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/spin.hpp"
#include "edyn/math/math.hpp"
#include "edyn/util/constraint_util.hpp"
#include <entt/entt.hpp>

namespace edyn {

void spin_angle_constraint::init(entt::entity entity, constraint &con, entt::registry &registry) {
    auto row_entity = add_constraint_row(entity, con, registry, 400);
    auto &row = registry.get<constraint_row>(row_entity);
    row.use_spin[0] = true;
    row.use_spin[1] = true;
    
    m_offset_origin = calculate_offset(con, registry);
}

void spin_angle_constraint::prepare(entt::entity, constraint &con, entt::registry &registry, scalar dt) {
    auto &ornA = registry.get<orientation>(con.body[0]);
    auto &ornB = registry.get<orientation>(con.body[1]);
    auto &spinA = registry.get<spin>(con.body[0]);
    auto &spinB = registry.get<spin>(con.body[1]);

    auto axisA = quaternion_x(ornA);
    auto axisB = quaternion_x(ornB);

    auto error = calculate_offset(con, registry) - m_offset_origin;
    auto relvel = spinA - spinB * m_ratio;
    auto force = error * m_stiffness + relvel * m_damping;
    auto impulse = std::abs(force) * dt;

    auto &row = registry.get<constraint_row>(con.row[0]);
    row.J = {vector3_zero, axisA, vector3_zero, -axisB * m_ratio};
    row.error = error / dt;
    row.lower_limit = -impulse;
    row.upper_limit = impulse;
}

void spin_angle_constraint::set_ratio(scalar ratio, const constraint &con, const entt::registry &registry) {
    m_ratio = ratio;
    m_offset_origin = calculate_offset(con, registry);
}

scalar spin_angle_constraint::calculate_offset(const constraint &con, const entt::registry &registry) const {
    auto &sA = registry.get<spin_angle>(con.body[0]);
    auto &sB = registry.get<spin_angle>(con.body[1]);
    return (sA.count - sB.count * m_ratio) * pi2 + (sA.s - sB.s * m_ratio);
}

}