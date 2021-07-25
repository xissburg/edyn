#include "edyn/constraints/antiroll_constraint.hpp"
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
void prepare_constraints<antiroll_constraint>(entt::registry &registry, row_cache &cache, scalar dt) {
    auto body_view = registry.view<position, orientation,
                                   linvel, angvel,
                                   mass_inv, inertia_world_inv,
                                   delta_linvel, delta_angvel>();
    auto con_view = registry.view<antiroll_constraint>();
    auto imp_view = registry.view<constraint_impulse>();

    con_view.each([&] (entt::entity entity, antiroll_constraint &con) {
        auto [pA, qA, linvelA, angvelA, inv_mA, inv_IA, dvA, dwA] =
            body_view.get<position, orientation, linvel, angvel, mass_inv, inertia_world_inv, delta_linvel, delta_angvel>(con.body[0]);
        auto [pB, qB, linvelB, angvelB, inv_mB, inv_IB, dvB, dwB] =
            body_view.get<position, orientation, linvel, angvel, mass_inv, inertia_world_inv, delta_linvel, delta_angvel>(con.body[1]);

        auto rA = rotate(qA, con.pivotA);
        auto posA = pA + rA;
        auto rB = rotate(qB, con.ctrl_arm_pivotB);
        auto posB = pB + rB;

        auto &pC = registry.get<position>(con.third_entity);
        auto &qC = registry.get<orientation>(con.third_entity);
        auto rC = rotate(qC, con.other_ctrl_arm_pivotC);
        auto posC = pC + rC;

        // Z axis points forward.
        auto chassis_z = rotate(qA, vector3_z);

        // Calculate control arm direction vector to build basis.
        auto ctrl_armA = pA + rotate(qA, con.ctrl_arm_pivotA);
        auto ctrl_armB = posB;
        auto ctrl_arm_dir = ctrl_armB - ctrl_armA;
        auto ctrl_arm_len = length(ctrl_arm_dir);
        ctrl_arm_dir /= ctrl_arm_len;

        // Calculate pivot point on control arm using basis.
        scalar side = con.ctrl_arm_pivotA.x > 0 ? 1 : -1;
        auto ctrl_arm_x = ctrl_arm_dir * side;
        auto ctrl_arm_y = cross(chassis_z, ctrl_arm_x);
        auto ctrl_arm_basis = matrix3x3_columns(ctrl_arm_x, ctrl_arm_y, chassis_z);
        auto ctrl_arm_pivot_rel = ctrl_arm_basis * con.ctrl_arm_pivot;
        auto ctrl_arm_pivot = ctrl_armA + ctrl_arm_pivot_rel;

        // Do the same for the control arm on the other side.
        auto other_ctrl_armA = pA + rotate(qA, con.other_ctrl_arm_pivotA);
        auto other_ctrl_armC = posC;
        auto other_ctrl_arm_dir = other_ctrl_armA - other_ctrl_armC;
        auto other_ctrl_arm_len = length(other_ctrl_arm_dir);
        other_ctrl_arm_dir /= other_ctrl_arm_len;

        auto other_ctrl_arm_x = other_ctrl_arm_dir * side;
        auto other_ctrl_arm_y = cross(chassis_z, other_ctrl_arm_x);
        auto other_ctrl_arm_basis = matrix3x3_columns(other_ctrl_arm_x, other_ctrl_arm_y, chassis_z);
        auto other_ctrl_arm_pivot_rel = other_ctrl_arm_basis * con.other_ctrl_arm_pivot;
        auto other_ctrl_arm_pivot = other_ctrl_armA + other_ctrl_arm_pivot_rel;

        auto dB = ctrl_arm_pivot - posA;
        auto dC = other_ctrl_arm_pivot - posA;
        auto chassis_x = rotate(qA, vector3_x);
        auto d_projB = dB - chassis_x * dot(dB, chassis_x);
        auto d_projC = dC - chassis_x * dot(dC, chassis_x);

        auto lever = std::max(length(d_projB), EDYN_EPSILON);
        d_projB /= lever;

        d_projC = normalize(d_projC);

        // Apply impulses in the direction of deformation.
        auto n = d_projC - d_projB;

        if (length_sqr(n) <= EDYN_EPSILON) {
            n = cross(d_projC, chassis_x);
        }

        n = normalize(n);

        auto p = cross(rA, n);
        auto q = cross(rB, n);
        auto d = std::clamp(dot(d_projB, d_projC), scalar(-1), scalar(1));
        auto angle = to_degrees(std::acos(d));
        auto impulse = std::abs(con.stiffness * angle / lever) * dt;

        auto &row = cache.rows.emplace_back();
        row.J = {n, p, -n, -q};
        row.lower_limit = -impulse;
        row.upper_limit = impulse;

        auto options = constraint_row_options{};
        options.error = angle / dt;

        row.inv_mA = inv_mA; row.inv_IA = inv_IA;
        row.inv_mB = inv_mB; row.inv_IB = inv_IB;
        row.dvA = &dvA; row.dwA = &dwA;
        row.dvB = &dvB; row.dwB = &dwB;
        row.impulse = imp_view.get(entity).values[0];

        prepare_row(row, options, linvelA, linvelB, angvelA, angvelB);
        warm_start(row);

        cache.con_num_rows.push_back(1);
    });
}

}