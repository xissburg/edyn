#include "edyn/constraints/triple_spin_constraint.hpp"
#include "edyn/dynamics/row_cache.hpp"
#include "edyn/comp/orientation.hpp"

namespace edyn {

void triple_spin_constraint::prepare(
    const entt::registry &, entt::entity,
    constraint_row_prep_cache &cache, scalar dt,
    const constraint_body &bodyA, const constraint_body &bodyB, const constraint_body &bodyC) {

    auto axis0 = rotate(bodyA.orn, m_axis[0]);
    auto axis1 = rotate(bodyB.orn, m_axis[1]);
    auto axis2 = rotate(bodyC.orn, m_axis[2]);

    auto &row = cache.add_row_triple();
    row.J = {vector3_zero, axis0 * m_ratio[0],
             vector3_zero, axis1 * m_ratio[1],
             vector3_zero, axis2 * m_ratio[2]};
    row.lower_limit = -large_scalar;
    row.upper_limit = large_scalar;

    row.impulse = applied_impulse;
    row.use_spin = m_use_spin;
    row.spin_axis[0] = axis0;
    row.spin_axis[1] = axis1;
    row.spin_axis[2] = axis2;
}

void triple_spin_constraint::store_applied_impulses(const std::vector<scalar> &impulses) {
    applied_impulse = impulses[0];
}

}
