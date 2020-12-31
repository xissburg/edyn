#include "edyn/collision/collide.hpp"
#include "edyn/math/geom.hpp"

namespace edyn {

collision_result collide(const sphere_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const box_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold) {

    // Sphere position and orientation in box space.
    const auto ornB_conj = conjugate(ornB);
    const auto posA_in_B = rotate(ornB_conj, posA - posB);
    const auto ornA_in_B = ornB_conj * ornA;

    // Inspired by Bullet's btSphereBoxCollisionAlgorithm.
    auto closest = closest_point_box_outside(shB.half_extents, posA_in_B);
    auto normalB = posA_in_B - closest;
    auto d_sqr = length_sqr(normalB);
    auto min_dist = shA.radius + threshold;

    if (d_sqr > min_dist * min_dist) {
        return {};
    }

    scalar center_distance;

    // If `posA_in_B` lies inside the box, `closest_point_box_outside`
    // returns `posA_in_B`.
    if (d_sqr <= EDYN_EPSILON) {
        // Sphere center lies inside box.
        center_distance = -closest_point_box_inside(shB.half_extents, posA_in_B, closest, normalB);
    } else {
        center_distance = std::sqrt(d_sqr);
        normalB /= center_distance;
    }

    auto pivotA_in_B = posA_in_B - normalB * shA.radius;

    collision_result result;
    result.num_points = 1;
    result.point[0].pivotA = to_object_space(pivotA_in_B, posA_in_B, ornA_in_B);
    result.point[0].pivotB = closest;
    result.point[0].normalB = normalB;
    result.point[0].distance = center_distance - shA.radius;

    return result;
}

}