#include "edyn/collision/collide.hpp"

namespace edyn {

collision_result collide(const sphere_shape &sphere, const plane_shape &plane, 
                         const collision_context &ctx) {
    auto normal = rotate(ctx.ornB, plane.normal);
    auto center = ctx.posB + rotate(ctx.ornB, plane.normal * plane.constant);
    auto d = ctx.posA - center;
    auto l = dot(normal, d);

    if (l > sphere.radius) {
        return {};
    }

    auto result = collision_result {};
    result.num_points = 1;
    result.point[0].pivotA = rotate(conjugate(ctx.ornA), -normal * sphere.radius);
    result.point[0].pivotB = rotate(conjugate(ctx.ornB), d - normal * l - center);
    result.point[0].normalB = plane.normal;
    result.point[0].distance = l - sphere.radius;
    return result;
}

collision_result collide(const plane_shape &shA, const sphere_shape &shB,
                         const collision_context &ctx) {
    return swap_collide(shA, shB, ctx);
}

}
