#include "edyn/collision/collide.hpp"

namespace edyn {

collision_result collide(const cylinder_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const plane_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold) {
    auto normal = rotate(ornB, shB.normal);
    auto pt = shA.support_point(posA, ornA, -normal);
    auto center = posB + normal * shB.constant;
    auto dist = dot(pt - center, normal);

    if (dist > threshold) {
        return {};
    }

    auto result = collision_result{};
    result.num_points = 1;
    result.point[0].pivotA = rotate(conjugate(ornA), pt - posA);
    result.point[0].pivotB = rotate(conjugate(ornB), pt - normal * dist - posB);
    result.point[0].normalB = shB.normal;
    
    return result;
}

}