#include "edyn/collision/collide.hpp"

namespace edyn {

static
void add_contact_points(scalar l, const vector3 &d, scalar radius, const vector3 &hl,
                        const quaternion &ornA, const vector3 &center, const quaternion &ornB,
                        const vector3 &normalB, const vector3 &world_normal, 
                        collision_result &result) {
    if (l <= radius) {
        auto idx = result.num_points++;
        result.point[idx].pivotA = rotate(conjugate(ornA), hl - world_normal * radius);
        result.point[idx].pivotB = rotate(conjugate(ornB), d - world_normal * l - center);
        result.point[idx].normalB = normalB;
        result.point[idx].distance = l - radius;
    }
}

collision_result collide(const capsule_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const plane_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold) {
    // Plane center and normal in world-space.
    auto normal = rotate(ornB, shB.normal);
    auto center = posB + rotate(ornB, shB.normal * shB.constant);

    // Half-length vector in world-space.
    auto hl = rotate(ornA, vector3_x * shA.half_length);

    // Center of hemispheres on capsule's ends.
    auto p0 = posA - hl;
    auto p1 = posA + hl;

    // Vector from hemisphere center to plane center.
    auto d0 = p0 - center;
    auto d1 = p1 - center;

    // Projection on plane normal.
    auto l0 = dot(normal, d0);
    auto l1 = dot(normal, d1);

    auto result = collision_result {};
    
    add_contact_points(l0, d0, shA.radius, -hl, ornA, center, ornB, shB.normal, normal, result);
    add_contact_points(l1, d1, shA.radius, hl, ornA, center, ornB, shB.normal, normal, result);

    return result;
}

}
