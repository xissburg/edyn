#include "edyn/collision/collide.hpp"

namespace edyn {

void collide(const sphere_shape &sphere, const plane_shape &plane, 
             const collision_context &ctx, collision_result &result) {
    auto normal = rotate(ctx.ornB, plane.normal);
    auto center = ctx.posB + rotate(ctx.ornB, plane.normal * plane.constant);
    auto d = ctx.posA - center;
    auto l = dot(normal, d);

    if (l > sphere.radius) {
        return;
    }

    auto pivotA = rotate(conjugate(ctx.ornA), -normal * sphere.radius);
    auto pivotB = rotate(conjugate(ctx.ornB), d - normal * l - center);
    auto normalB = plane.normal;
    auto distance = l - sphere.radius;
    result.add_point({pivotA, pivotB, normalB, distance});
}

void collide(const plane_shape &shA, const sphere_shape &shB,
             const collision_context &ctx, collision_result &result) {
    swap_collide(shA, shB, ctx, result);
}

}
