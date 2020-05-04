#include "edyn/collision/collide.hpp"

namespace edyn {

collision_result collide(const sphere_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const sphere_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold) {
    auto d = posA - posB;
    auto l2 = length_sqr(d);
    auto r = shA.radius + shB.radius + threshold;

    if (l2 > r * r) {
        return {};
    }

    auto l = std::sqrt(l2);
    auto dn = d / l;
    auto rA = -dn * shA.radius;
    rA = rotate(conjugate(ornA), rA);
    auto rB = dn * shB.radius;
    rB = rotate(conjugate(ornB), rB);

    auto result = collision_result {};
    result.num_points = 1;
    result.point[0].pivotA = rA;
    result.point[0].pivotB = rB;
    result.point[0].normalB = rotate(conjugate(ornB), dn);
    result.point[0].distance = l - shA.radius - shB.radius;
    return result;
}

}