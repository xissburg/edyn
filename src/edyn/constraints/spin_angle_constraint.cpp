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

template<>
void prepare_constraints<spin_angle_constraint>(entt::registry &registry, row_cache &cache, scalar dt) {
    auto body_view = registry.view<position, orientation,
                                   linvel, angvel, spin,
                                   mass_inv, inertia_world_inv,
                                   delta_linvel, delta_angvel, delta_spin>();
    auto con_view = registry.view<spin_angle_constraint>();

    con_view.each([&] (entt::entity entity, spin_angle_constraint &con) {
        if (std::abs(con.m_ratio) < EDYN_EPSILON) {
            cache.con_num_rows.push_back(0);
            return;
        }

        auto [posA, ornA, linvelA, angvelA, spinA, inv_mA, inv_IA, dvA, dwA, dsA] = body_view.get(con.body[0]);
        auto [posB, ornB, linvelB, angvelB, spinB, inv_mB, inv_IB, dvB, dwB, dsB] = body_view.get(con.body[1]);

        auto axisA = quaternion_x(ornA);
        auto axisB = quaternion_x(ornB);

        auto spinvelA = axisA * spinA;
        auto spinvelB = axisB * spinB;

        {
            auto error = con.calculate_offset(registry) - con.m_offset_origin;
            auto force = error * con.m_stiffness;
            auto impulse = std::abs(force) * dt;

            auto &row = cache.rows.emplace_back();
            row.J = {vector3_zero, axisA, vector3_zero, -axisB * con.m_ratio};
            row.lower_limit = -impulse;
            row.upper_limit = impulse;
            row.inv_mA = inv_mA; row.inv_IA = inv_IA;
            row.inv_mB = inv_mB; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dwA = &dwA; row.dsA = &dsA;
            row.dvB = &dvB; row.dwB = &dwB; row.dsB = &dsB;
            row.impulse = con.impulse[0];
            row.use_spin[0] = true;
            row.use_spin[1] = true;
            row.spin_axis[0] = axisA;
            row.spin_axis[1] = axisB;

            auto options = constraint_row_options{};
            options.error = error / dt;

            prepare_row(row, options, linvelA, angvelA + spinvelA, linvelB, angvelB + spinvelB);
            warm_start(row);
        }

        {
            auto relvel = spinA - spinB * con.m_ratio;
            auto force = relvel * con.m_damping;
            auto impulse = std::abs(force) * dt;

            auto &row = cache.rows.emplace_back();
            row.J = {vector3_zero, axisA, vector3_zero, -axisB * con.m_ratio};
            row.lower_limit = -impulse;
            row.upper_limit = impulse;
            row.inv_mA = inv_mA; row.inv_IA = inv_IA;
            row.inv_mB = inv_mB; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dwA = &dwA; row.dsA = &dsA;
            row.dvB = &dvB; row.dwB = &dwB; row.dsB = &dsB;
            row.impulse = con.impulse[1];
            row.use_spin[0] = true;
            row.use_spin[1] = true;
            row.spin_axis[0] = axisA;
            row.spin_axis[1] = axisB;

            prepare_row(row, {}, linvelA, angvelA + spinvelA, linvelB, angvelB + spinvelB);
            warm_start(row);
        }

        cache.con_num_rows.push_back(2);
    });
}

void spin_angle_constraint::set_ratio(scalar ratio, const entt::registry &registry) {
    if (ratio == m_ratio) {
        return;
    }

    m_ratio = ratio;
    m_offset_origin = calculate_offset(registry);
}

scalar spin_angle_constraint::calculate_offset(const entt::registry &registry) const {
    auto &sA = registry.get<spin_angle>(body[0]);
    auto &sB = registry.get<spin_angle>(body[1]);
    return (sA.count - sB.count * m_ratio) * pi2 + (sA.s - sB.s * m_ratio);
}

}
