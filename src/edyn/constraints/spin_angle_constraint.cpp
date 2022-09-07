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
    const vector3 &originA, const vector3 &posA, const quaternion &ornA,
    const vector3 &linvelA, const vector3 &angvelA,
    const vector3 &originB, const vector3 &posB, const quaternion &ornB,
    const vector3 &linvelB, const vector3 &angvelB) {

    if (std::abs(m_ratio) < EDYN_EPSILON) {
        return;
    }

    auto axisA = quaternion_x(ornA);
    auto axisB = quaternion_x(ornB);

    auto spinvelA = axisA * spinA;
    auto spinvelB = axisB * spinB;

    {
        auto error = calculate_offset(registry) - m_offset_origin;
        auto force = error * m_stiffness;
        auto impulse = std::abs(force) * dt;

        auto &row = cache.add_row();
        row.J = {vector3_zero, axisA, vector3_zero, -axisB * m_ratio};
        row.lower_limit = -impulse;
        row.upper_limit = impulse;
        row.impulse = impulse[0];
        row.use_spin[0] = true;
        row.use_spin[1] = true;
        row.spin_axis[0] = axisA;
        row.spin_axis[1] = axisB;

        cache.get_options().error = error / dt;
    }

    {
        auto relvel = spinA - spinB * m_ratio;
        auto force = relvel * m_damping;
        auto impulse = std::abs(force) * dt;

        auto &row = cache.add_row();
        row.J = {vector3_zero, axisA, vector3_zero, -axisB * m_ratio};
        row.lower_limit = -impulse;
        row.upper_limit = impulse;
        row.impulse = impulse[1];
        row.use_spin[0] = true;
        row.use_spin[1] = true;
        row.spin_axis[0] = axisA;
        row.spin_axis[1] = axisB;
    }
}

void spin_angle_constraint::set_ratio(scalar ratio, const entt::registry &registry) {
    m_ratio = ratio;
    m_offset_origin = calculate_offset(registry);
}

scalar spin_angle_constraint::calculate_offset(const entt::registry &registry) const {
    auto &sA = registry.get<spin_angle>(body[0]);
    auto &sB = registry.get<spin_angle>(body[1]);
    return (sA.count - sB.count * m_ratio) * pi2 + (sA.s - sB.s * m_ratio);
}

}
