#include "edyn/collision/collide.hpp"
#include "edyn/math/geom.hpp"

namespace edyn {

collision_result collide(const capsule_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const capsule_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold) {
    // Half-length vector in world-space.
    auto hlA = rotate(ornA, vector3_x * shA.half_length);
    auto hlB = rotate(ornB, vector3_x * shB.half_length);

    // Center of hemispheres on capsule's ends.
    auto p0A = posA - hlA;
    auto p1A = posA + hlA;

    auto p0B = posB - hlB;
    auto p1B = posB + hlB;

    scalar s, t, sp, tp;
    vector3 cA, cB, cAp, cBp;
    size_t num_points;

    auto l2 = closest_point_segment_segment(p0A, p1A, p0B, p1B, s, t, cA, cB, 
                                            &num_points, &sp, &tp, &cAp, &cBp);
    auto r = shA.radius + shB.radius + threshold;
    
    if (l2 > r * r) {
        return {};
    }

    auto l = std::sqrt(l2);
    vector3 dn;

    if (l2 > EDYN_EPSILON) {
        auto d = cA - cB;
        dn = d / l;
    } else {
        dn = vector3_y;
    }

    auto rA = (cA - dn * shA.radius) - posA;
    auto rB = (cB + dn * shB.radius) - posB;

    auto result = collision_result {};
    result.num_points = num_points;
    result.point[0].pivotA = rotate(conjugate(ornA), rA);
    result.point[0].pivotB = rotate(conjugate(ornB), rB);
    result.point[0].normalB = rotate(conjugate(ornB), dn);
    result.point[0].distance = l - shA.radius - shB.radius;

    if (num_points > 1) {
        auto rAp = (cAp - dn * shA.radius) - posA;
        auto rBp = (cBp + dn * shB.radius) - posB;

        result.point[1].pivotA = rotate(conjugate(ornA), rAp);
        result.point[1].pivotB = rotate(conjugate(ornB), rBp);
        result.point[1].normalB = rotate(conjugate(ornB), dn);
        result.point[1].distance = l - shA.radius - shB.radius;
    }

    return result;
}

}
