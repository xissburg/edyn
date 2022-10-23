#include "edyn/constraints/tirecarcass_constraint.hpp"
#include "edyn/constraints/constraint_row.hpp"
#include "edyn/dynamics/row_cache.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/delta_linvel.hpp"
#include "edyn/comp/delta_angvel.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/comp/spin.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/util/constraint_util.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

void tirecarcass_constraint::prepare(
    const entt::registry &, entt::entity,
    constraint_row_prep_cache &cache, scalar dt,
    const constraint_body &bodyA, const constraint_body &bodyB) {

    const auto axisA_x = quaternion_x(bodyA.orn);

    const auto axisB_x = quaternion_x(bodyB.orn);
    const auto axisB_y = quaternion_y(bodyB.orn);
    const auto axisB_z = quaternion_z(bodyB.orn);

    // Lateral movement.
    {
        auto error = dot(bodyA.pos - bodyB.pos, axisB_x);
        auto spring_force = error * m_lateral_stiffness;
        auto spring_impulse = std::abs(spring_force * dt);

        auto &row = cache.add_row();
        row.J = {axisB_x, vector3_zero, -axisB_x, vector3_zero};
        row.lower_limit = -spring_impulse;
        row.upper_limit = spring_impulse;
        row.impulse = applied_impulse.lateral_spring;

        cache.get_options().error = error / dt;
    }

    // Lateral damping.
    {
        auto relspd = dot(bodyA.linvel - bodyB.linvel, axisB_x);
        auto damping_force = m_lateral_damping * relspd;
        auto damping_impulse = std::abs(damping_force * dt);

        auto &row = cache.add_row();
        row.J = {axisB_x, vector3_zero, -axisB_x, vector3_zero};
        row.lower_limit = -damping_impulse;
        row.upper_limit = damping_impulse;
        row.impulse = applied_impulse.lateral_damping;
    }

    // Prevent vertical movement.
    {
        auto &row = cache.add_row();
        row.J = {axisB_y, vector3_zero, -axisB_y, vector3_zero};
        row.lower_limit = -large_scalar;
        row.upper_limit = large_scalar;
        row.impulse = applied_impulse.vertical;

        cache.get_options().error = dot(bodyA.pos - bodyB.pos, axisB_y) / dt;
    }

    // Prevent longitudinal movement.
    {
        auto &row = cache.add_row();
        row.J = {axisB_z, vector3_zero, -axisB_z, vector3_zero};
        row.lower_limit = -large_scalar;
        row.upper_limit = large_scalar;
        row.impulse = applied_impulse.longitudinal;

        cache.get_options().error = dot(bodyA.pos - bodyB.pos, axisB_z) / dt;
    }

    // Constrain rotation along all axes.
    for (size_t i = 0; i < 3; ++i) {
        constexpr auto I = matrix3x3_identity;
        auto axis = rotate(bodyA.orn, I.row[i]);
        auto n = rotate(bodyA.orn, I.row[(i+1)%3]);
        auto m = rotate(bodyB.orn, I.row[(i+2)%3]);
        auto error = dot(n, m);

        auto &row = cache.add_row();
        row.J = {vector3_zero, axis, vector3_zero, -axis};
        row.lower_limit = -large_scalar;
        row.upper_limit = large_scalar;
        row.impulse = applied_impulse.rotational[i];

        cache.get_options().error = error / dt;
    }

    // Longitudinal twist.
    {
        auto error = (bodyA.spin_angle - bodyB.spin_angle) + (bodyA.spin_count - bodyB.spin_count) * pi2;
        auto spring_torque = error * m_longitudinal_stiffness;
        auto spring_impulse = std::abs(spring_torque * dt);

        auto &row = cache.add_row_with_spin();
        row.J = {vector3_zero, axisA_x, vector3_zero, -axisB_x};
        row.lower_limit = -spring_impulse;
        row.upper_limit = spring_impulse;
        row.impulse = applied_impulse.longitudinal_twist_spring;
        row.use_spin[0] = true;
        row.use_spin[1] = true;
        row.spin_axis[0] = axisA_x;
        row.spin_axis[1] = axisB_x;

        cache.get_options().error = error / dt;
    }

    // Longitudinal damping.
    {
        auto relspd = bodyA.spin - bodyB.spin;
        auto damping_torque = m_longitudinal_damping * relspd;
        auto damping_impulse = std::abs(damping_torque * dt);

        auto &row = cache.add_row_with_spin();
        row.J = {vector3_zero, axisA_x, vector3_zero, -axisB_x};
        row.lower_limit = -damping_impulse;
        row.upper_limit = damping_impulse;
        row.impulse = applied_impulse.longitudinal_twist_damping;
        row.use_spin[0] = true;
        row.use_spin[1] = true;
        row.spin_axis[0] = axisA_x;
        row.spin_axis[1] = axisB_x;
    }
}

void tirecarcass_constraint::store_applied_impulses(const std::vector<scalar> &impulses) {
    unsigned row_idx = 0;
    applied_impulse.lateral_spring = impulses[row_idx++];
    applied_impulse.lateral_damping = impulses[row_idx++];
    applied_impulse.vertical = impulses[row_idx++];
    applied_impulse.longitudinal = impulses[row_idx++];

    for (size_t i = 0; i < 3; ++i) {
        applied_impulse.rotational[i] = impulses[row_idx++];
    }

    applied_impulse.longitudinal_twist_spring = impulses[row_idx++];
    applied_impulse.longitudinal_twist_damping = impulses[row_idx++];
}

}
