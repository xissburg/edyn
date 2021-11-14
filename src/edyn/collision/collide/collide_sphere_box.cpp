#include "edyn/collision/collide.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/math/transform.hpp"

namespace edyn {

void collide(const sphere_shape &shA, const box_shape &shB,
             const collision_context &ctx, collision_result &result) {
    // Sphere position and orientation in box space.
    const auto ornB_conj = conjugate(ctx.ornB);
    const auto posA_in_B = rotate(ornB_conj, ctx.posA - ctx.posB);
    const auto ornA_in_B = ornB_conj * ctx.ornA;

    // Inspired by Bullet's btSphereBoxCollisionAlgorithm.
    auto closest = closest_point_box_outside(shB.half_extents, posA_in_B);
    auto normalB = posA_in_B - closest;
    auto d_sqr = length_sqr(normalB);
    auto min_dist = shA.radius + ctx.threshold;

    if (d_sqr > min_dist * min_dist) {
        return;
    }

    scalar center_distance;
    auto normal_attachment = contact_normal_attachment::none;

    // If `posA_in_B` lies inside the box, `closest_point_box_outside`
    // returns `posA_in_B`.
    if (d_sqr <= EDYN_EPSILON) {
        // Sphere center lies inside box.
        center_distance = -closest_point_box_inside(shB.half_extents, posA_in_B, closest, normalB);
        normal_attachment = contact_normal_attachment::normal_on_B;
    } else {
        center_distance = std::sqrt(d_sqr);
        normalB /= center_distance;

        // If the local normal is aligned with any of the coordinate axes, it
        // means the closest feature on the box is a face and thus the normal
        // should be attached to the box.
        if (std::abs(normalB.x) > scalar(1) - EDYN_EPSILON ||
            std::abs(normalB.y) > scalar(1) - EDYN_EPSILON ||
            std::abs(normalB.z) > scalar(1) - EDYN_EPSILON)
        {
            normal_attachment = contact_normal_attachment::normal_on_B;
        }
    }

    auto pivotA_in_B = posA_in_B - normalB * shA.radius;

    auto pivotA = to_object_space(pivotA_in_B, posA_in_B, ornA_in_B);
    auto pivotB = closest;
    auto normal = rotate(ctx.ornB, normalB);
    auto distance = center_distance - shA.radius;
    result.add_point({pivotA, pivotB, normal, distance, normal_attachment});
}

void collide(const box_shape &shA, const sphere_shape &shB,
             const collision_context &ctx, collision_result &result) {
    swap_collide(shA, shB, ctx, result);
}

}
