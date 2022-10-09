#include "edyn/constraints/spin_constraint.hpp"
#include "edyn/constraints/constraint_row.hpp"
#include "edyn/dynamics/row_cache.hpp"
#include "edyn/comp/spin.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/delta_linvel.hpp"
#include "edyn/comp/delta_angvel.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/util/constraint_util.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

void spin_constraint::prepare(const entt::registry &registry, entt::entity entity,
                              constraint_row_prep_cache &cache, scalar dt,
                              const constraint_body &bodyA, const constraint_body &bodyB) {
    auto axisA = rotate(bodyA.orn, vector3_x);
    auto axisB = rotate(bodyB.orn, vector3_x);

    auto spinvelA = vector3_zero;
    auto spinvelB = vector3_zero;

    if (m_use_spinA) {
        spinvelA = axisA * bodyA.spin;
    }

    if (m_use_spinB) {
        spinvelB = axisB * bodyB.spin;
    }

    auto impulse = m_max_torque * dt;

    auto &row = cache.add_row_with_spin();
    row.J = {vector3_zero, axisA, vector3_zero, -axisB};
    row.lower_limit = -impulse;
    row.upper_limit = impulse;
    row.impulse = impulse;
    row.use_spin[0] = m_use_spinA;
    row.use_spin[1] = m_use_spinB;
    row.spin_axis[0] = axisA;
    row.spin_axis[1] = axisB;
}

}
