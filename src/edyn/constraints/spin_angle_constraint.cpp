#include "edyn/constraints/spin_angle_constraint.hpp"
#include "edyn/constraints/constraint_row.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/spin.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/delta_linvel.hpp"
#include "edyn/comp/delta_angvel.hpp"
#include "edyn/dynamics/row_cache.hpp"
#include "edyn/util/constraint_util.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

void spin_angle_constraint::prepare(
    const entt::registry &registry, entt::entity entity,
    constraint_row_prep_cache &cache, scalar dt,
    const constraint_body &bodyA, const constraint_body &bodyB) {

    if (std::abs(m_ratio) < EDYN_EPSILON) {
        return;
    }

    auto axisA = quaternion_x(bodyA.orn);
    auto axisB = quaternion_x(bodyB.orn);

    if (m_engaged) {
        auto error = calculate_offset(registry) - m_offset_origin;
        auto force = error * m_stiffness;
        auto impulse = std::abs(force) * dt;

        auto &row = cache.add_row_with_spin();
        row.J = {vector3_zero, axisA, vector3_zero, -axisB * m_ratio};
        row.lower_limit = -impulse;
        row.upper_limit = impulse;
        row.impulse = applied_impulse.spring;
        row.use_spin[0] = true;
        row.use_spin[1] = true;
        row.spin_axis[0] = axisA;
        row.spin_axis[1] = axisB;

        cache.get_options().error = error / dt;
    }

    if (m_engaged) {
        auto relvel = bodyA.spin - bodyB.spin * m_ratio;
        auto force = relvel * m_damping;
        auto impulse = std::abs(force) * dt;

        auto &row = cache.add_row_with_spin();
        row.J = {vector3_zero, axisA, vector3_zero, -axisB * m_ratio};
        row.lower_limit = -impulse;
        row.upper_limit = impulse;
        row.impulse = applied_impulse.damping;
        row.use_spin[0] = true;
        row.use_spin[1] = true;
        row.spin_axis[0] = axisA;
        row.spin_axis[1] = axisB;
    }

    if (!m_engaged) {
        auto impulse = std::abs(m_friction_torque) * dt;

        auto &row = cache.add_row_with_spin();
        row.J = {vector3_zero, axisA, vector3_zero, -axisB * m_ratio};
        row.lower_limit = -impulse;
        row.upper_limit = impulse;
        row.impulse = applied_impulse.friction;
        row.use_spin[0] = true;
        row.use_spin[1] = true;
        row.spin_axis[0] = axisA;
        row.spin_axis[1] = axisB;
    }
}

void spin_angle_constraint::set_engaged(bool engaged, const entt::registry &registry) {
    if (m_engaged == engaged) {
        return;
    }

    m_engaged = engaged;

    if (m_engaged) {
        m_offset_origin = calculate_offset(registry);
    }
}

void spin_angle_constraint::set_ratio(scalar ratio, const entt::registry &registry) {
    m_ratio = ratio;

    if (m_engaged) {
        m_offset_origin = calculate_offset(registry);
    }
}

scalar spin_angle_constraint::calculate_offset(const entt::registry &registry) const {
    auto &sA = registry.get<spin_angle>(body[0]);
    auto &sB = registry.get<spin_angle>(body[1]);
    return (sA.count - sB.count * m_ratio) * pi2 + (sA.s - sB.s * m_ratio);
}

scalar spin_angle_constraint::relative_velocity(const entt::registry &registry) const {
    auto &sA = registry.get<spin>(body[0]);
    auto &sB = registry.get<spin>(body[1]);
    return sA.s - sB.s * m_ratio;
}

void spin_angle_constraint::store_applied_impulses(const std::vector<scalar> &impulses) {
    if (impulses.empty()) {
        return;
    }

    if (m_engaged) {
        EDYN_ASSERT(impulses.size() == 2);
        applied_impulse.spring = impulses[0];
        applied_impulse.damping = impulses[1];
        applied_impulse.friction = 0;
    } else {
        applied_impulse.spring = 0;
        applied_impulse.damping = 0;
        applied_impulse.friction = impulses[0];
    }
}

}
