#include "edyn/constraints/doublewishbone_constraint.hpp"
#include "edyn/constraints/constraint_row.hpp"
#include "edyn/constraints/constraint_impulse.hpp"
#include "edyn/dynamics/row_cache.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/delta_linvel.hpp"
#include "edyn/comp/delta_angvel.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/math/math.hpp"
#include "edyn/util/constraint_util.hpp"
#include <entt/entt.hpp>

namespace edyn {

template<>
void prepare_constraints<doublewishbone_constraint>(entt::registry &registry, row_cache &cache, scalar dt) {
    auto body_view = registry.view<position, orientation,
                                   linvel, angvel,
                                   mass_inv, inertia_world_inv,
                                   delta_linvel, delta_angvel>();
    auto con_view = registry.view<doublewishbone_constraint>();
    auto imp_view = registry.view<constraint_impulse>();

    con_view.each([&] (entt::entity entity, doublewishbone_constraint &con) {
        auto [posA, ornA, linvelA, angvelA, inv_mA, inv_IA, dvA, dwA] =
            body_view.get<position, orientation, linvel, angvel, mass_inv, inertia_world_inv, delta_linvel, delta_angvel>(con.body[0]);
        auto [posB, ornB, linvelB, angvelB, inv_mB, inv_IB, dvB, dwB] =
            body_view.get<position, orientation, linvel, angvel, mass_inv, inertia_world_inv, delta_linvel, delta_angvel>(con.body[1]);
        auto &imp = imp_view.get(entity);

        // Upper control arm locations.
        auto urA = rotate(ornA, con.upper_pivotA);
        auto uposA = posA + urA;

        auto urB = rotate(ornB, con.upper_pivotB);
        auto uposB = posB + urB;

        auto ud = uposA - uposB;
        auto ul2 = length_sqr(ud);

        // Lower control arm locations.
        auto lrA = rotate(ornA, con.lower_pivotA);
        auto lposA = posA + lrA;

        auto lrB = rotate(ornB, con.lower_pivotB);
        auto lposB = posB + lrB;

        auto ld = lposA - lposB;
        auto ll2 = length_sqr(ld);

        // Z axis points forward.
        auto chassis_z = rotate(ornA, vector3_z);

        // Wheel rotation axis.
        auto wheel_x = rotate(ornB, vector3_x * con.side);

        auto row_idx = size_t(0);

        // Upper control arm distance constraint.
        {
            auto &row = cache.rows.emplace_back();
            row.J = {ud, cross(urA, ud), -ud, -cross(urB, ud)};
            row.lower_limit = -large_scalar;
            row.upper_limit = large_scalar;

            row.inv_mA = inv_mA;  row.inv_IA = inv_IA;
            row.inv_mB = inv_mB; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dwA = &dwA;
            row.dvB = &dvB; row.dwB = &dwB;
            row.impulse = imp.values[row_idx++];

            auto options = constraint_row_options{};
            options.error = 0.5 * (ul2 - con.upper_length * con.upper_length) / dt;

            prepare_row(row, options, linvelA, linvelB, angvelA, angvelB);
            warm_start(row);
        }

        // Lower control arm distance constraint
        {
            auto &row = cache.rows.emplace_back();
            row.J = {ld, cross(lrA, ld), -ld, -cross(lrB, ld)};
            row.lower_limit = -large_scalar;
            row.upper_limit = large_scalar;

            row.inv_mA = inv_mA;  row.inv_IA = inv_IA;
            row.inv_mB = inv_mB; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dwA = &dwA;
            row.dvB = &dvB; row.dwB = &dwB;
            row.impulse = imp.values[row_idx++];

            auto options = constraint_row_options{};
            options.error = 0.5 * (ll2 - con.lower_length * con.lower_length) / dt;

            prepare_row(row, options, linvelA, linvelB, angvelA, angvelB);
            warm_start(row);
        }

        // Constrain upper pivot on wheel to a plane that passes through upper pivot
        // on chassis with normal equals chassis' z axis
        {
            auto p = cross(urA, chassis_z) + cross(chassis_z, ud);
            auto q = cross(urB, chassis_z);

            auto &row = cache.rows.emplace_back();
            row.J = {chassis_z, p, -chassis_z, -q};
            row.lower_limit = -large_scalar;
            row.upper_limit = large_scalar;

            row.inv_mA = inv_mA;  row.inv_IA = inv_IA;
            row.inv_mB = inv_mB; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dwA = &dwA;
            row.dvB = &dvB; row.dwB = &dwB;
            row.impulse = imp.values[row_idx++];

            auto options = constraint_row_options{};
            options.error = dot(ud, chassis_z) / dt;

            prepare_row(row, options, linvelA, linvelB, angvelA, angvelB);
            warm_start(row);
        }

        // Constrain lower pivot on wheel to a plane that passes through lower pivot
        // on chassis with normal equals chassis' z axis
        {
            auto p = cross(lrA, chassis_z) + cross(chassis_z, ld);
            auto q = cross(lrB, chassis_z);

            auto &row = cache.rows.emplace_back();
            row.J = {chassis_z, p, -chassis_z, -q};
            row.lower_limit = -large_scalar;
            row.upper_limit = large_scalar;

            row.inv_mA = inv_mA;  row.inv_IA = inv_IA;
            row.inv_mB = inv_mB; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dwA = &dwA;
            row.dvB = &dvB; row.dwB = &dwB;
            row.impulse = imp.values[row_idx++];

            auto options = constraint_row_options{};
            options.error = dot(ld, chassis_z) / dt;

            prepare_row(row, options, linvelA, linvelB, angvelA, angvelB);
            warm_start(row);
        }

        auto mrA = (urA + lrA) / 2;
        auto mrB = (urB + lrB) / 2;
        auto mposA = (uposA + lposA) / 2;
        auto mposB = (uposB + lposB) / 2;
        auto md = mposA - mposB;

        // Constrain the middle of the axis on the wheel to always stay in front of
        // a plane passing through the middle of the axis on the chassis with normal
        // pointing outside the vehicle.
        {
            auto chassis_x = rotate(ornA, vector3_x * con.side);
            auto p = cross(mrA, chassis_x) + cross(chassis_x, md);
            auto q = cross(mrB, chassis_x);

            auto &row = cache.rows.emplace_back();
            row.J = {chassis_x, p, -chassis_x, -q};
            row.lower_limit = -large_scalar;
            row.upper_limit = 0;

            row.inv_mA = inv_mA;  row.inv_IA = inv_IA;
            row.inv_mB = inv_mB; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dwA = &dwA;
            row.dvB = &dvB; row.dwB = &dwB;
            row.impulse = imp.values[row_idx++];

            auto options = constraint_row_options{};
            options.error = 0.2 * (dot(md, chassis_x) + 0.2) / dt; // be gentle

            prepare_row(row, options, linvelA, linvelB, angvelA, angvelB);
            warm_start(row);
        }

        if (!con.steerable) {
            // Constrain wheel rotation axis to a plane that passes through upper pivot
            // on chassis with normal equals chassis' z axis
            auto q = cross(chassis_z, wheel_x);

            auto &row = cache.rows.emplace_back();
            row.J = {vector3_zero, q, vector3_zero, -q};
            row.lower_limit = -large_scalar;
            row.upper_limit = large_scalar;

            row.inv_mA = inv_mA;  row.inv_IA = inv_IA;
            row.inv_mB = inv_mB; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dwA = &dwA;
            row.dvB = &dvB; row.dwB = &dwB;
            row.impulse = imp.values[row_idx++];

            auto options = constraint_row_options{};
            options.error = dot(chassis_z, wheel_x) / dt;

            prepare_row(row, options, linvelA, linvelB, angvelA, angvelB);
            warm_start(row);
        }

        cache.con_num_rows.push_back(row_idx);
    });
}

}