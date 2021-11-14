#include "edyn/constraints/antiroll_constraint.hpp"
#include "edyn/comp/origin.hpp"
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
#include "edyn/comp/center_of_mass.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/math/math.hpp"
#include "edyn/math/transform.hpp"
#include "edyn/util/constraint_util.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

template<>
void prepare_constraints<antiroll_constraint>(entt::registry &registry, row_cache &cache, scalar dt) {
    auto body_view = registry.view<position, orientation,
                                   linvel, angvel,
                                   mass_inv, inertia_world_inv,
                                   delta_linvel, delta_angvel>();
    auto con_view = registry.view<antiroll_constraint>();
    auto imp_view = registry.view<constraint_impulse>();
    auto origin_view = registry.view<origin>();

    con_view.each([&] (entt::entity entity, antiroll_constraint &con) {
        auto [posA, ornA, linvelA, angvelA, inv_mA, inv_IA, dvA, dwA] =
            body_view.get<position, orientation, linvel, angvel, mass_inv, inertia_world_inv, delta_linvel, delta_angvel>(con.body[0]);
        auto [posB, ornB, linvelB, angvelB, inv_mB, inv_IB, dvB, dwB] =
            body_view.get<position, orientation, linvel, angvel, mass_inv, inertia_world_inv, delta_linvel, delta_angvel>(con.body[1]);
        auto [posC, ornC] = body_view.get<position, orientation>(con.m_third_entity);

        auto originA = origin_view.contains(con.body[0]) ? origin_view.get<origin>(con.body[0]) : static_cast<vector3>(posA);
        auto originB = origin_view.contains(con.body[1]) ? origin_view.get<origin>(con.body[1]) : static_cast<vector3>(posB);
        auto originC = origin_view.contains(con.m_third_entity) ? origin_view.get<origin>(con.m_third_entity) : static_cast<vector3>(posC);

        auto ctrl_armA = to_world_space(con.m_ctrl_arm_pivotA, originA, ornA);
        auto ctrl_armB = to_world_space(con.m_ctrl_arm_pivotB, originB, ornB);

        // Calculate control arm direction vector to build basis.
        auto ctrl_arm_dir = ctrl_armB - ctrl_armA;
        auto ctrl_arm_len = length(ctrl_arm_dir);
        ctrl_arm_dir /= ctrl_arm_len;

        auto rA = ctrl_armB - posA;
        auto rB = ctrl_armB - posB;

        // Z axis points forward.
        auto chassis_z = quaternion_z(ornA);
        // X axis points to the left.
        auto chassis_x = quaternion_x(ornA);

        // Calculate pivot point on control arm using basis.
        scalar side = con.m_ctrl_arm_pivotA.x > 0 ? 1 : -1;
        auto ctrl_arm_x = ctrl_arm_dir * side;
        auto ctrl_arm_y = cross(chassis_z, ctrl_arm_x);
        auto ctrl_arm_basis = matrix3x3_columns(ctrl_arm_x, ctrl_arm_y, chassis_z);
        auto ctrl_arm_pivot_rel = ctrl_arm_basis * con.m_ctrl_arm_pivot;
        auto ctrl_arm_pivot = ctrl_armA + ctrl_arm_pivot_rel;

        // Do the same for the control arm on the other side.
        auto other_ctrl_armA = to_world_space(con.m_other_ctrl_arm_pivotA, originA, ornA);
        auto other_ctrl_armC = to_world_space(con.m_other_ctrl_arm_pivotC, originC, ornC);
        auto other_ctrl_arm_dir = other_ctrl_armA - other_ctrl_armC;
        auto other_ctrl_arm_len = length(other_ctrl_arm_dir);
        other_ctrl_arm_dir /= other_ctrl_arm_len;

        auto other_ctrl_arm_x = other_ctrl_arm_dir * side;
        auto other_ctrl_arm_y = cross(chassis_z, other_ctrl_arm_x);
        auto other_ctrl_arm_basis = matrix3x3_columns(other_ctrl_arm_x, other_ctrl_arm_y, chassis_z);
        auto other_ctrl_arm_pivot_rel = other_ctrl_arm_basis * con.m_other_ctrl_arm_pivot;
        auto other_ctrl_arm_pivot = other_ctrl_armA + other_ctrl_arm_pivot_rel;

        auto pivotA = to_world_space(con.m_pivotA, originA, ornA);
        auto leverB = project_direction(ctrl_arm_pivot - pivotA, chassis_x);
        auto leverC = project_direction(other_ctrl_arm_pivot - pivotA, chassis_x);

        auto lever_lenB = length(leverB);
        auto lever_lenC = length(leverC);

        EDYN_ASSERT(lever_lenB > EDYN_EPSILON);
        EDYN_ASSERT(lever_lenC > EDYN_EPSILON);

        leverB /= lever_lenB;
        leverC /= lever_lenC;

        // Force is generated in the direction of lever arm deflection, which
        // attempts to make the angle betweem them go to zero.
        auto force_dir = leverC - leverB;
        auto force_dir_len_sqr = length_sqr(force_dir);

        if (force_dir_len_sqr > EDYN_EPSILON) {
            force_dir /= std::sqrt(force_dir_len_sqr);
        } else {
            force_dir = cross(leverC, chassis_x);
        }

        auto angle = to_degrees(std::acos(std::clamp(dot(leverB, leverC), scalar(-1), scalar(1))));
        auto torque = con.m_stiffness * angle;
        auto force = torque / lever_lenB;

        // Apply corrective impulse at the wheel pivot along the direction
        // normal to the control arm, similar to springdamper_constraint.
        auto cos_theta = dot(ctrl_arm_y, force_dir);
        auto ctrl_arm_pivot_horizontal_dist = con.m_ctrl_arm_pivot.x * side;
        auto ctrl_arm_pivot_ratio = ctrl_arm_pivot_horizontal_dist / ctrl_arm_len;
        auto ctrl_arm_pivot_ratio_inv = scalar(1) / ctrl_arm_pivot_ratio;
        auto lever_term = ctrl_arm_pivot_ratio * cos_theta;
        auto impulse = std::abs(force * lever_term) * dt;

        auto d = ctrl_arm_y;
        auto p = cross(rA, d);
        auto q = cross(rB, d);

        auto &row = cache.rows.emplace_back();
        row.J = {d, p, -d, -q};
        row.lower_limit = -impulse;
        row.upper_limit = impulse;

        auto options = constraint_row_options{};
        // Make error inversely proportional to distance from control arm pivot.
        options.error = angle * cos_theta * ctrl_arm_pivot_ratio_inv / dt;

        row.inv_mA = inv_mA; row.inv_IA = inv_IA;
        row.inv_mB = inv_mB; row.inv_IB = inv_IB;
        row.dvA = &dvA; row.dwA = &dwA;
        row.dvB = &dvB; row.dwB = &dwB;
        row.impulse = imp_view.get<constraint_impulse>(entity).values[0];

        prepare_row(row, options, linvelA, angvelA, linvelB, angvelB);
        warm_start(row);

        cache.con_num_rows.push_back(1);
    });
}

}