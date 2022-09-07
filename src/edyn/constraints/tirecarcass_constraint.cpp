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
    const vector3 &originA, const vector3 &posA, const quaternion &ornA,
    const vector3 &linvelA, const vector3 &angvelA,
    const vector3 &originB, const vector3 &posB, const quaternion &ornB,
    const vector3 &linvelB, const vector3 &angvelB) {

    const auto axisA_x = quaternion_x(ornA);

    const auto axisB_x = quaternion_x(ornB);
    const auto axisB_y = quaternion_y(ornB);
    const auto axisB_z = quaternion_z(ornB);

    auto spinvelA = axisA_x * spinA;
    auto spinvelB = axisB_x * spinB;

    auto row_idx = size_t(0);

    // Lateral movement.
    {
        auto error = dot(posA - posB, axisB_x);
        auto spring_force = -error * m_lateral_stiffness;
        auto spring_impulse = std::abs(spring_force * dt);

        auto &row = cache.add_row();
        row.J = {axisB_x, vector3_zero, -axisB_x, vector3_zero};
        row.lower_limit = -spring_impulse;
        row.upper_limit = spring_impulse;
        row.impulse = impulse[row_idx++];

        cache.get_options().error = error / dt;
    }

    // Lateral damping.
    {
        auto relspd = dot(linvelA - linvelB, axisB_x);
        auto damping_force = m_lateral_damping * relspd;
        auto damping_impulse = std::abs(damping_force * dt);

        auto &row = cache.add_row();
        row.J = {axisB_x, vector3_zero, -axisB_x, vector3_zero};
        row.lower_limit = -damping_impulse;
        row.upper_limit = damping_impulse;
        row.impulse = impulse[row_idx++];
    }

    // Prevent vertical movement.
    {
        auto &row = cache.add_row();
        row.J = {axisB_y, vector3_zero, -axisB_y, vector3_zero};
        row.lower_limit = -large_scalar;
        row.upper_limit = large_scalar;
        row.impulse = impulse[row_idx++];

        cache.get_options().error = dot(posA - posB, axisB_y) / dt;
    }

    // Prevent longitudinal movement.
    {
        auto &row = cache.add_row();
        row.J = {axisB_z, vector3_zero, -axisB_z, vector3_zero};
        row.lower_limit = -large_scalar;
        row.upper_limit = large_scalar;
        row.impulse = impulse[row_idx++];

        cache.get_options().error = dot(posA - posB, axisB_z) / dt;
    }

    // Constrain rotation along all axes.
    for (size_t i = 0; i < 3; ++i) {
        constexpr auto I = matrix3x3_identity;
        auto axis = rotate(ornA, I.row[i]);
        auto n = rotate(ornA, I.row[(i+1)%3]);
        auto m = rotate(ornB, I.row[(i+2)%3]);
        auto error = dot(n, m);

        auto &row = cache.add_row();
        row.J = {vector3_zero, axis, vector3_zero, -axis};
        row.lower_limit = -large_scalar;
        row.upper_limit = large_scalar;
        row.impulse = impulse[row_idx++];

        cache.get_options().error = error / dt;
    }

    // Longitudinal twist.
    {
        auto error = (angleA.s - angleB.s) + (angleA.count - angleB.count) * pi2;
        auto spring_torque = -error * m_longitudinal_stiffness;
        auto spring_impulse = spring_torque * dt;
        auto impulse = std::abs(spring_impulse);

        auto &row = cache.add_row();
        row.J = {vector3_zero, axisA_x, vector3_zero, -axisB_x};
        row.lower_limit = -impulse;
        row.upper_limit = impulse;
        row.impulse = impulse[row_idx++];
        row.use_spin[0] = true;
        row.use_spin[1] = true;
        row.spin_axis[0] = axisA_x;
        row.spin_axis[1] = axisB_x;

        cache.get_options().error = error / dt;
    }

    // Longitudinal damping.
    {
        auto relspd = spinA.s - spinB.s;
        auto damping_torque = m_longitudinal_damping * relspd;
        auto damping_impulse = damping_torque * dt;
        auto impulse = std::abs(damping_impulse);

        auto &row = cache.add_row();
        row.J = {vector3_zero, axisA_x, vector3_zero, -axisB_x};
        row.lower_limit = -impulse;
        row.upper_limit =  impulse;
        row.use_spin[0] = true;
        row.use_spin[1] = true;
        row.spin_axis[0] = axisA_x;
        row.spin_axis[1] = axisB_x;
        row.impulse = impulse[row_idx++];
    }
}

}
