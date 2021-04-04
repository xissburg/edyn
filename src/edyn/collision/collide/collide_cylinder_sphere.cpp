#include "edyn/collision/collide.hpp"
#include "edyn/math/geom.hpp"

namespace edyn {

collision_result collide(const cylinder_shape &shA, const sphere_shape &shB, 
                         const collision_context &ctx) {
    const auto &posA = ctx.posA;
    const auto &ornA = ctx.ornA;
    const auto &posB = ctx.posB;
    const auto &ornB = ctx.ornB;
    const auto threshold = ctx.threshold;

    const auto hl = rotate(ornA, vector3_x * shA.half_length);
    const auto p0 = posA - hl;
    const auto p1 = posA + hl;

    const auto v = p1 - p0; // Direction vector of cylinder segment.
    const auto w = posB - p0; // Vector from initial point of cylinder segment to point sphere center.
    const auto a = dot(w, v);
    const auto b = dot(v, v);
    EDYN_ASSERT(b > EDYN_EPSILON);
    const auto t = a / b;

    if (t > 0 && t < 1) {
        const auto p = p0 + v * t;
        const auto d = p - posB;
        const auto l2 = length_sqr(d);
        const auto min_dist = shA.radius + shB.radius + threshold;
    
        if (l2 > min_dist * min_dist) {
            return {};
        }

        const auto l = std::sqrt(l2);
        const auto normal = l2 > EDYN_EPSILON ? d / l : vector3_y;
        auto result = collision_result{};
        result.num_points = 1;
        result.point[0].pivotA = rotate(conjugate(ornA), p - normal * shA.radius - posA);
        result.point[0].pivotB = rotate(conjugate(ornB), normal * shB.radius);
        result.point[0].normalB = rotate(conjugate(ornB), normal);
        result.point[0].distance = l - shA.radius - shB.radius;
        return result;
    }

    const auto dpos = t < 0.5 ? p0 : p1;
    vector3 q;
    const auto l2 = closest_point_disc(dpos, ornA, shA.radius, posB, q);
    const auto min_dist = shB.radius + threshold;

    if (l2 > min_dist * min_dist) {
        return {};
    }

    auto normal = q - posB;
    const auto nl2 = length_sqr(normal);
    const auto nl = std::sqrt(nl2);
    normal = nl2 > EDYN_EPSILON ? normal / nl : rotate(ornA, vector3_x) * (t < 0.5 ? -1 : 1);

    auto result = collision_result{};
    result.num_points = 1;
    result.point[0].pivotA = rotate(conjugate(ornA), q - posA);
    result.point[0].pivotB = rotate(conjugate(ornB), normal * shB.radius);
    result.point[0].normalB = rotate(conjugate(ornB), normal);
    result.point[0].distance = nl - shB.radius;
    return result;
}

collision_result collide(const sphere_shape &shA, const cylinder_shape &shB,
                         const collision_context &ctx) {
    return swap_collide(shA, shB, ctx);
}

}
