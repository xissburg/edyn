#include "edyn/constraints/tirecarcass_constraint.hpp"
#include "edyn/constraints/constraint_row.hpp"
#include "edyn/constraints/constraint_impulse.hpp"
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

template<>
void prepare_constraints<tirecarcass_constraint>(entt::registry &registry, row_cache &cache, scalar dt) {
    auto body_view = registry.view<position, orientation, spin_angle,
                                   linvel, angvel, spin,
                                   mass_inv, inertia_world_inv,
                                   delta_linvel, delta_angvel, delta_spin>();
    auto con_view = registry.view<tirecarcass_constraint>();
    auto imp_view = registry.view<constraint_impulse>();

    con_view.each([&] (entt::entity entity, tirecarcass_constraint &con) {
        auto [posA, ornA, angleA, linvelA, angvelA, spinA, inv_mA, inv_IA, dvA, dwA, dsA] =
            body_view.get<position, orientation, spin_angle, linvel, angvel, spin, mass_inv, inertia_world_inv, delta_linvel, delta_angvel, delta_spin>(con.body[0]);
        auto [posB, ornB, angleB, linvelB, angvelB, spinB, inv_mB, inv_IB, dvB, dwB, dsB] =
            body_view.get<position, orientation, spin_angle, linvel, angvel, spin, mass_inv, inertia_world_inv, delta_linvel, delta_angvel, delta_spin>(con.body[1]);
        auto &imp = imp_view.get(entity);

        const auto axisA_x = quaternion_x(ornA);
        const auto axisA_y = quaternion_y(ornA);
        const auto axisA_z = quaternion_z(ornA);

        const auto axisB_x = quaternion_x(ornB);
        const auto axisB_y = quaternion_y(ornB);
        const auto axisB_z = quaternion_z(ornB);

        auto spinvelA = axisA_x * spinA;
        auto spinvelB = axisB_x * spinB;

        auto row_idx = size_t(0);

        // Lateral movement.
        {
            auto error = -dot(posA - posB, axisB_x);
            auto spring_force = error * con.m_lateral_stiffness;
            auto spring_impulse = spring_force * dt;

            auto &row = cache.rows.emplace_back();
            row.J = {axisB_x, vector3_zero, -axisB_x, vector3_zero};
            row.lower_limit = -large_scalar;
            row.upper_limit = large_scalar;

            auto options = constraint_row_options{};
            options.error = -error / dt;

            row.inv_mA = inv_mA; row.inv_IA = inv_IA;
            row.inv_mB = inv_mB; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dwA = &dwA;
            row.dvB = &dvB; row.dwB = &dwB;
            row.impulse = imp.values[row_idx++];

            prepare_row(row, options, linvelA, linvelB, angvelA, angvelB);
            warm_start(row);
        }

        // Prevent vertical movement.
        {
            auto &row = cache.rows.emplace_back();
            row.J = {axisB_y, vector3_zero, -axisB_y, vector3_zero};
            row.lower_limit = -large_scalar;
            row.upper_limit = large_scalar;

            auto options = constraint_row_options{};
            options.error = dot(posA - posB, axisB_y) / dt;

            row.inv_mA = inv_mA; row.inv_IA = inv_IA;
            row.inv_mB = inv_mB; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dwA = &dwA;
            row.dvB = &dvB; row.dwB = &dwB;
            row.impulse = imp.values[row_idx++];

            prepare_row(row, options, linvelA, linvelB, angvelA, angvelB);
            warm_start(row);
        }

        // Prevent backwards/forwards movement.
        {
            auto &row = cache.rows.emplace_back();
            row.J = {axisB_z, vector3_zero, -axisB_z, vector3_zero};
            row.lower_limit = -large_scalar;
            row.upper_limit = large_scalar;

            auto options = constraint_row_options{};
            options.error = dot(posA - posB, axisB_z) / dt;

            row.inv_mA = inv_mA; row.inv_IA = inv_IA;
            row.inv_mB = inv_mB; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dwA = &dwA;
            row.dvB = &dvB; row.dwB = &dwB;
            row.impulse = imp.values[row_idx++];

            prepare_row(row, options, linvelA, linvelB, angvelA, angvelB);
            warm_start(row);
        }

        // Prevent rotation along spin axis.
        {
            auto &row = cache.rows.emplace_back();
            row.J = {vector3_zero, axisB_x, vector3_zero, -axisB_x};
            row.lower_limit = -large_scalar;
            row.upper_limit = large_scalar;

            auto options = constraint_row_options{};
            options.error = dot(axisA_y, axisB_z) / dt;

            row.inv_mA = inv_mA; row.inv_IA = inv_IA;
            row.inv_mB = inv_mB; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dwA = &dwA;
            row.dvB = &dvB; row.dwB = &dwB;
            row.impulse = imp.values[row_idx++];

            prepare_row(row, options, linvelA, linvelB, angvelA, angvelB);
            warm_start(row);
        }

        // Torsional.
        {
            auto error = -dot(axisB_x, axisA_z);
            auto spring_force = error * con.m_torsional_stiffness;
            auto spring_impulse = spring_force * dt;

            auto &row = cache.rows.emplace_back();
            row.J = {vector3_zero, axisB_y, vector3_zero, -axisB_y};
            row.lower_limit = -large_scalar;
            row.upper_limit = large_scalar;

            auto options = constraint_row_options{};
            options.error = dot(axisA_z, axisB_x) / dt;

            row.inv_mA = inv_mA; row.inv_IA = inv_IA;
            row.inv_mB = inv_mB; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dwA = &dwA;
            row.dvB = &dvB; row.dwB = &dwB;
            row.impulse = imp.values[row_idx++];

            prepare_row(row, options, linvelA, linvelB, angvelA, angvelB);
            warm_start(row);
        }

        // Prevent rolling (rotation along forward axis).
        {
            auto &row = cache.rows.emplace_back();
            row.J = {vector3_zero, axisB_z, vector3_zero, -axisB_z};
            row.lower_limit = -large_scalar;
            row.upper_limit = large_scalar;

            auto options = constraint_row_options{};
            options.error = (half_pi - std::acos(dot(axisA_x, axisB_y))) / dt;

            row.inv_mA = inv_mA; row.inv_IA = inv_IA;
            row.inv_mB = inv_mB; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dwA = &dwA;
            row.dvB = &dvB; row.dwB = &dwB;
            row.impulse = imp.values[row_idx++];

            prepare_row(row, options, linvelA, linvelB, angvelA, angvelB);
            warm_start(row);
        }

        // Longitudinal twist.
        {
            auto error = (angleA.s - angleB.s) + (angleA.count - angleB.count) * pi2;
            auto spring_torque = -error * con.m_longitudinal_stiffness;
            auto spring_impulse = spring_torque * dt;

            auto &row = cache.rows.emplace_back();
            row.J = {vector3_zero, axisA_x, vector3_zero, -axisB_x};
            row.lower_limit = -large_scalar;
            row.upper_limit = large_scalar;
            row.use_spin[0] = true;
            row.use_spin[1] = true;
            row.spin_axis[0] = axisA_x;
            row.spin_axis[1] = axisB_x;
            row.inv_mA = inv_mA; row.inv_IA = inv_IA;
            row.inv_mB = inv_mB; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dwA = &dwA; row.dsA = &dsA;
            row.dvB = &dvB; row.dwB = &dwB; row.dsB = &dsB;
            row.impulse = imp.values[row_idx++];

            auto options = constraint_row_options{};
            options.error = error / dt;

            prepare_row(row, options, linvelA, linvelB, angvelA + spinvelA, angvelB + spinvelB);
            warm_start(row);
        }

        cache.con_num_rows.push_back(row_idx);
    });
}

}
