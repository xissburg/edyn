#include "edyn/constraints/tierod_constraint.hpp"
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
#include "edyn/comp/center_of_mass.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/math/math.hpp"
#include "edyn/util/constraint_util.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

template<>
void prepare_constraints<tierod_constraint>(entt::registry &registry, row_cache &cache, scalar dt) {
    auto body_view = registry.view<position, orientation,
                                   linvel, angvel,
                                   mass_inv, inertia_world_inv,
                                   delta_linvel, delta_angvel>();
    auto con_view = registry.view<tierod_constraint>();
    auto imp_view = registry.view<constraint_impulse>();
    auto com_view = registry.view<center_of_mass>();

    con_view.each([&] (entt::entity entity, tierod_constraint &con) {
        auto [posA, ornA, linvelA, angvelA, inv_mA, inv_IA, dvA, dwA] =
            body_view.get<position, orientation, linvel, angvel, mass_inv, inertia_world_inv, delta_linvel, delta_angvel>(con.body[0]);
        auto [posB, ornB, linvelB, angvelB, inv_mB, inv_IB, dvB, dwB] =
            body_view.get<position, orientation, linvel, angvel, mass_inv, inertia_world_inv, delta_linvel, delta_angvel>(con.body[1]);

        con.update_steering_axis();
        con.update_steering_arm();

        auto originA = static_cast<vector3>(posA);
        auto originB = static_cast<vector3>(posB);

        if (com_view.contains(con.body[0])) {
            auto &com = com_view.get(con.body[0]);
            originA = to_world_space(-com, posA, ornA);
        }

        if (com_view.contains(con.body[1])) {
            auto &com = com_view.get(con.body[1]);
            originB = to_world_space(-com, posB, ornB);
        }

        // Given the tie rod joint position on the steering rack (pivotA), the position
        // of the upper and lower control arm joints on the upright, the position of
        // the tie rod joint on the upright in object space, and the tie rod length,
        // determine the world space position of the tie rod joint on the upright.
        // It's a circle-circle intersection problem thus we make a projection on the
        // plane of the circle where pivotA lies on the x axis. We want one point on
        // the steering arm circle which has `rod_length` distance to pivotA.
        // Then rotate a steering arm vector according to this joint position which
        // must be orthogonal to the wheel spin axis and use this vector in a constraint
        // row which forces the wheel spin axis to be orthogonal to this vector.
        auto pivotA = to_world_space(con.pivotA + con.pivotA_offset, originA, ornA);

        // Calculate origin of the steering arm circle in world space. It's a point
        // in the steering axis. Originally, this origin would be calculated as an
        // offset from the lower control arm pivot point on the wheel, that is:
        // axis_z = normalize(steering_axis);
        // vector3 lower_pivot = originB + rotate(ornB, lower_pivotB);
        // vector3 origin = dot(pivotB - lower_pivotB, axis_z) * axis_z + lower_pivot;
        // Due to the wheel not always exactly matching the control arm pivot points
        // due to non zero constraint error, this calculation would place the origin
        // on an axis on the wheel which would lead to an incorrect steering angle
        // calculation. That's noticeable when the car is spinning. To fix this it
        // is necessary to calculate the origin as an offset from a calculated lower
        // pivot point using the fixed lower control arm length, which also needs to
        // be projected along the chassis Z axis to counter sideways error. This way,
        // the steering axis is placed where the control arm should be so the
        // constraint error (from the double wishbone constraint) should have minimal
        // effect in this calculation.

        auto chassis_z = quaternion_z(ornA);

        auto upA = to_world_space(con.upper_pivotA, originA, ornA);
        auto upB = to_world_space(con.upper_pivotB, originB, ornB);
        auto upper_dir = upB - upA;
        upper_dir -= chassis_z * dot(upper_dir, chassis_z);
        upper_dir = normalize(upper_dir);
        auto upper_pivot = upA + upper_dir * con.upper_length;

        auto lpA = to_world_space(con.lower_pivotA, originA, ornA);
        auto lpB = to_world_space(con.lower_pivotB, originB, ornB);
        auto lower_dir = lpB - lpA;
        lower_dir -= chassis_z * dot(lower_dir, chassis_z);
        lower_dir = normalize(lower_dir);
        auto lower_pivot = lpA + lower_dir * con.lower_length;

        auto axis_z = normalize(upper_pivot - lower_pivot); // world-space steering axis
        auto origin = lower_pivot + axis_z * dot(con.pivotB - con.lower_pivotB, con.steering_axis);

        // Distance to the plane that contains the steering arm circle.
        auto dist_plane = dot(pivotA - origin, axis_z);
        auto pivotA_proj = pivotA - axis_z * dist_plane;

        // x-axis for the steering basis.
        auto axis_x = pivotA_proj - origin;
        auto dist_origin = length(axis_x);
        axis_x /= dist_origin;

        // The length of the tie rod projected on the plane space on the steering
        // arm circle.
        auto proj_len_sq = std::max(scalar(0), con.rod_length * con.rod_length - dist_plane * dist_plane);
        auto proj_len = std::sqrt(proj_len_sq);

        // Find solution to the circle-circle intersection on the 2D plane. Our
        // projection puts `pivotA` on the x-axis which simplifies the formulas.
        // There are two solutions to this problem on y but we only care about
        // the positive y.
        scalar x = 1, y = 0;

        // If `pivotA` is too close or too far from the steering axis a solution
        // does not exist. Thus use the closest point on the circle.
        if (dist_origin >= proj_len + con.steering_arm_length) {
            x = con.steering_arm_length;
        } else if (dist_origin <= proj_len - con.steering_arm_length) {
            x = -con.steering_arm_length;
        } else { // calculate solution from derived formula
            auto r = con.steering_arm_length;
            auto d = proj_len;
            auto c = dist_origin;
            x = (r * r + c * c - d * d) / (2 * c);
            // If `pivotB` lies in front of the upright we have to choose negative y.
            auto y_sq = r * r - x * x;
            EDYN_ASSERT(y_sq >= 0);
            y = std::sqrt(y_sq) * (con.pivotB.z > 0 ? -1 : 1);
        }

        // This is the vector the wheel spin axis should be orthogonal to.
        // It needs to be rotated so that it lies on the upright plane.
        auto n = vector3 {x, y, 0};
        n = rotate(quaternion_axis_angle(vector3_z, con.steering_arm_angle), n);

        scalar side = con.pivotA.x > 0 ? 1 : -1;
        auto axis_y = cross(axis_x, axis_z) * side;
        auto basis = matrix3x3_columns(axis_x, axis_y, axis_z);
        n = normalize(basis * n);

        auto wheel_x = quaternion_x(ornB);
        auto q = cross(n, wheel_x);

        auto &row = cache.rows.emplace_back();
        row.J = {vector3_zero, q, vector3_zero, -q};
        row.lower_limit = -large_scalar;
        row.upper_limit = large_scalar;

        auto options = constraint_row_options{};
        options.error = dot(n, wheel_x) / dt;

        row.inv_mA = inv_mA; row.inv_IA = inv_IA;
        row.inv_mB = inv_mB; row.inv_IB = inv_IB;
        row.dvA = &dvA; row.dwA = &dwA;
        row.dvB = &dvB; row.dwB = &dwB;
        row.impulse = imp_view.get(entity).values[0];

        prepare_row(row, options, linvelA, angvelA, linvelB, angvelB);
        warm_start(row);

        cache.con_num_rows.push_back(1);
    });
}

void tierod_constraint::update_steering_axis() {
    steering_axis = normalize(upper_pivotB - lower_pivotB);
}

void tierod_constraint::update_steering_arm() {
    auto v = pivotB - lower_pivotB;
    auto w = steering_axis * dot(v, steering_axis);
    auto p = lower_pivotB + w; // closest point on steering axis to pivotB
    steering_arm = pivotB - p;
    steering_arm_length = length(steering_arm);
    auto n = steering_arm / steering_arm_length;
    steering_arm_angle = std::acos(dot(n, -vector3_z));
}

}