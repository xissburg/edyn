#include "edyn/constraints/spin_constraint.hpp"
#include "edyn/dynamics/row_cache.hpp"
#include "edyn/math/vector3.hpp"

namespace edyn {

void spin_constraint::prepare(const entt::registry &, entt::entity,
                              constraint_row_prep_cache &cache, scalar dt,
                              const constraint_body &bodyA, const constraint_body &bodyB) {
    auto axisA = rotate(bodyA.orn, m_axisA);
    auto axisB = rotate(bodyB.orn, m_axisB);
    auto torque_impulse = m_max_torque * dt;

    auto &row = cache.add_row_with_spin();
    row.J = {vector3_zero, axisA, vector3_zero, -axisB};
    row.lower_limit = -torque_impulse;
    row.upper_limit = torque_impulse;
    row.impulse = applied_impulse;
    row.use_spin[0] = m_use_spinA;
    row.use_spin[1] = m_use_spinB;
    row.spin_axis[0] = axisA;
    row.spin_axis[1] = axisB;

    cache.get_options().error = m_target_spin;
}

void spin_constraint::store_applied_impulses(const std::vector<scalar> &impulses) {
    applied_impulse = impulses[0];
}

}
