#include "edyn/constraints/tierod_constraint.hpp"
#include "edyn/comp/origin.hpp"
#include "edyn/constraints/constraint_row.hpp"
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
#include "edyn/math/transform.hpp"
#include "edyn/util/constraint_util.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

void tierod_constraint::prepare(
    const entt::registry &registry, entt::entity entity,
    constraint_row_prep_cache &cache, scalar dt,
    const constraint_body &bodyA, const constraint_body &bodyB) {

    update_steering_axis();
    update_steering_arm();

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
    auto &offset = registry.get<tierod_offset>(entity);
    auto pivotA_world = to_world_space(pivotA + offset.value, bodyA.origin, bodyA.orn);

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

    auto chassis_z = quaternion_z(bodyA.orn);

    auto upA = to_world_space(upper_pivotA, bodyA.origin, bodyA.orn);
    auto upB = to_world_space(upper_pivotB, bodyB.origin, bodyB.orn);
    auto upper_dir = upB - upA;
    upper_dir -= chassis_z * dot(upper_dir, chassis_z);
    upper_dir = normalize(upper_dir);
    auto upper_pivot = upA + upper_dir * upper_length;

    auto lpA = to_world_space(lower_pivotA, bodyA.origin, bodyA.orn);
    auto lpB = to_world_space(lower_pivotB, bodyB.origin, bodyB.orn);
    auto lower_dir = lpB - lpA;
    lower_dir -= chassis_z * dot(lower_dir, chassis_z);
    lower_dir = normalize(lower_dir);
    auto lower_pivot = lpA + lower_dir * lower_length;

    auto axis_z = normalize(upper_pivot - lower_pivot); // world-space steering axis
    auto origin = lower_pivot + axis_z * dot(pivotB - lower_pivotB, steering_axis);

    // Distance to the plane that contains the steering arm circle.
    auto dist_plane = dot(pivotA_world - origin, axis_z);
    auto pivotA_proj = pivotA_world - axis_z * dist_plane;

    // x-axis for the steering basis.
    auto axis_x = pivotA_proj - origin;
    auto dist_origin = length(axis_x);
    axis_x /= dist_origin;

    // The length of the tie rod projected on the plane space on the steering
    // arm circle.
    auto proj_len_sq = std::max(scalar(0), rod_length * rod_length - dist_plane * dist_plane);
    auto proj_len = std::sqrt(proj_len_sq);

    // Find solution to the circle-circle intersection on the 2D plane. Our
    // projection puts `pivotA` on the x-axis which simplifies the formulas.
    // There are two solutions to this problem on y but we only care about
    // the positive y.
    scalar x = 1, y = 0;

    // If `pivotA` is too close or too far from the steering axis a solution
    // does not exist. Thus use the closest point on the circle.
    if (dist_origin >= proj_len + steering_arm_length) {
        x = steering_arm_length;
    } else if (dist_origin <= proj_len - steering_arm_length) {
        x = -steering_arm_length;
    } else { // calculate solution from derived formula
        auto r = steering_arm_length;
        auto d = proj_len;
        auto c = dist_origin;
        x = (r * r + c * c - d * d) / (2 * c);
        // If `pivotB` lies in front of the upright we have to choose negative y.
        auto y_sq = r * r - x * x;
        EDYN_ASSERT(y_sq >= 0);
        y = std::sqrt(y_sq) * (pivotB.z > 0 ? -1 : 1);
    }

    // This is the vector the wheel spin axis should be orthogonal to.
    // It needs to be rotated so that it lies on the upright plane.
    auto n = vector3 {x, y, 0};
    n = rotate(quaternion_axis_angle(vector3_z, steering_arm_angle), n);

    scalar side = pivotA.x > 0 ? 1 : -1;
    auto axis_y = cross(axis_x, axis_z) * side;
    auto basis = matrix3x3_columns(axis_x, axis_y, axis_z);
    n = normalize(basis * n);

    auto wheel_x = quaternion_x(bodyB.orn);
    auto q = cross(n, wheel_x);

    auto &row = cache.add_row();
    row.J = {vector3_zero, q, vector3_zero, -q};
    row.lower_limit = -large_scalar;
    row.upper_limit = large_scalar;
    row.impulse = applied_impulse;

    cache.get_options().error = dot(n, wheel_x) / dt;
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

void tierod_constraint::store_applied_impulses(const std::vector<scalar> &impulses) {
    applied_impulse = impulses[0];
}

}
