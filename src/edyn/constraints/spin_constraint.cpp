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

    auto spin_view = registry.view<spin, delta_spin>();
    auto con_view = registry.view<spin_constraint>();

    auto axisA = rotate(bodyA.orn, vector3_x);
    auto axisB = rotate(bodyB.orn, vector3_x);

    auto spinvelA = vector3_zero;
    auto spinvelB = vector3_zero;

    if (m_use_spinA) {
        auto spinA = spin_view.get<const spin>(body[0]);
        spinvelA = axisA * spinA;
    }

    if (m_use_spinB) {
        auto spinB = spin_view.get<const spin>(body[1]);
        spinvelB = axisB * spinB;
    }

    auto impulse = m_max_torque * dt;

    auto &row = cache.add_row();
    row.J = {vector3_zero, axisA, vector3_zero, -axisB};
    row.lower_limit = -impulse;
    row.upper_limit = impulse;
    row.impulse = impulse;
    row.use_spin[0] = m_use_spinA;
    row.use_spin[1] = m_use_spinB;
    row.spin_axis[0] = axisA;
    row.spin_axis[1] = axisB;

    if (m_use_spinA) {
        auto &dsA = spin_view.get<delta_spin>(body[0]);
        row.dsA = &dsA;
    } else {
        row.dsA = nullptr;
    }

    if (m_use_spinB) {
        auto &dsB = spin_view.get<delta_spin>(body[1]);
        row.dsB = &dsB;
    } else {
        row.dsB = nullptr;
    }
}

}
