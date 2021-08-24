#include "edyn/collision/collide.hpp"

namespace edyn {

void collide(const sphere_shape &sphere, const plane_shape &plane,
             const collision_context &ctx, collision_result &result) {
    auto normal = plane.normal;
    auto center = plane.normal * plane.constant;
    auto d = ctx.posA - center;
    auto l = dot(normal, d);

    if (l > sphere.radius) {
        return;
    }

    auto pivotA = rotate(conjugate(ctx.ornA), -normal * sphere.radius);
    auto pivotB = rotate(conjugate(ctx.ornB), d - normal * l - center);
    auto distance = l - sphere.radius;
    result.add_point({pivotA, pivotB, plane.normal, distance, contact_normal_attachment::normal_on_B});
}

void collide(const plane_shape &shA, const sphere_shape &shB,
             const collision_context &ctx, collision_result &result) {
    swap_collide(shA, shB, ctx, result);
}

}
