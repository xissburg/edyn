#include "edyn/constraints/cone_constraint.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/math/transform.hpp"
#include "edyn/math/vector2_3_util.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/dynamics/row_cache.hpp"
#include <entt/entity/registry.hpp>
#include <cmath>

namespace edyn {

void cone_constraint::prepare(
    const entt::registry &, entt::entity,
    constraint_row_prep_cache &cache, scalar dt,
    const constraint_body &bodyA, const constraint_body &bodyB) {

    // Transform B's pivot into the frame space in A and apply scaling so
    // that the cone is circular and has an opening angle of 90 degrees, which
    // makes calculations easier. The transformation is later reverted to
    // gather results in world space.
    auto pivotB_world = to_world_space(pivot[1], bodyB.origin, bodyB.orn);
    auto pivotB_in_A = to_object_space(pivotB_world, bodyA.origin, bodyA.orn);
    auto pivotB_in_A_frame = to_object_space(pivotB_in_A, pivot[0], frame);

    // Scaling to make the cone circular with an opening of 90 degrees.
    auto scaling_y = scalar(1) / span_tan[0];
    auto scaling_z = scalar(1) / span_tan[1];
    auto pivotB_in_A_frame_scaled = pivotB_in_A_frame * vector3{1, scaling_y, scaling_z};

    // Calculate normal vector on cone which points towards the pivot.
    auto proj_yz_len_sqr = length_sqr(to_vector2_yz(pivotB_in_A_frame_scaled));
    vector3 normal_scaled, tangent_scaled;

    if (proj_yz_len_sqr > EDYN_EPSILON) {
        normal_scaled = normalize(vector3{-std::sqrt(proj_yz_len_sqr),
                                            pivotB_in_A_frame_scaled.y,
                                            pivotB_in_A_frame_scaled.z});
        tangent_scaled = normalize(vector3{0, -pivotB_in_A_frame_scaled.z, pivotB_in_A_frame_scaled.y});
    } else {
        normal_scaled = normalize(vector3{-1, 1, 0});
        tangent_scaled = normalize(vector3{0, 0, 1});
    }

    auto error = dot(pivotB_in_A_frame_scaled, normal_scaled);

    // Find point cone closest to B's pivot.
    auto dir_on_cone = vector3{-normal_scaled.x, normal_scaled.y, normal_scaled.z};
    auto cone_proj = dot(pivotB_in_A_frame_scaled, dir_on_cone);
    auto point_on_cone_scaled = dir_on_cone * cone_proj;
    auto point_on_cone = point_on_cone_scaled * vector3{1, 1 / scaling_y, 1 / scaling_z};

    auto pivotA = to_world_space(point_on_cone, pivot[0], frame);
    auto pivotA_world = to_world_space(pivotA, bodyA.origin, bodyA.orn);

    // The tangent to a circle continues to be a tangent of the ellipse after
    // both are scaled, unlike the normal vector. Thus, descale the tangent
    // and recalculate the normal.
    auto tangent = normalize(tangent_scaled * vector3{1, 1 / scaling_y, 1 / scaling_z});
    auto normal = normalize(cross(tangent, point_on_cone));
    auto normal_world = rotate(bodyA.orn, frame * normal);

    auto rA = pivotA_world - bodyA.pos;
    auto rB = pivotB_world - bodyB.pos;

    std::array<vector3, 4> J =
        {normal_world,  cross(rA, normal_world),
        -normal_world, -cross(rB, normal_world)};

    /* Main row for the cone limits. */ {
        auto &row = cache.add_row();
        row.J = J;
        row.lower_limit = 0;
        row.upper_limit = large_scalar;
        row.impulse = limit_impulse;

        auto &options = cache.get_options();
        options.error = -error / dt;
        options.restitution = restitution;
    }

    if (bump_stop_stiffness > 0 && bump_stop_length > 0) {
        auto &row = cache.add_row();
        row.J = J;
        row.impulse = bump_stop_impulse;

        auto bump_stop_deflection = bump_stop_length + error;
        auto spring_force = bump_stop_stiffness * bump_stop_deflection;
        auto spring_impulse = spring_force * dt;
        row.lower_limit = 0;
        row.upper_limit = std::max(scalar(0), spring_impulse);

        auto &options = cache.get_options();
        options.error = -bump_stop_deflection / dt;
    }
}

void cone_constraint::store_applied_impulses(const std::vector<scalar> &impulses) {
    limit_impulse = impulses[0];

    if (bump_stop_stiffness > 0 && bump_stop_length > 0) {
        bump_stop_impulse = impulses[1];
    }
}

}
