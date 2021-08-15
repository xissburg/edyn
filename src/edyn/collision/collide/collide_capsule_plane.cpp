#include "edyn/collision/collide.hpp"

namespace edyn {

void collide(const capsule_shape &shA, const plane_shape &shB,
             const collision_context &ctx, collision_result &result) {
    auto center = shB.normal * shB.constant;
    auto capsule_vertices = shA.get_vertices(ctx.posA, ctx.ornA);

    for (auto &vertex : capsule_vertices) {
        auto distance = dot(vertex - center, shB.normal) - shA.radius;

        if (distance > ctx.threshold) continue;

        auto pivotA_world = vertex - shB.normal * shA.radius;
        auto pivotA = to_object_space(pivotA_world, ctx.posA, ctx.ornA);
        auto pivotB = project_plane(vertex, center, shB.normal);
        result.add_point({pivotA, pivotB, shB.normal, distance, contact_normal_attachment::normal_on_B});
    }
}

void collide(const plane_shape &shA, const capsule_shape &shB,
             const collision_context &ctx, collision_result &result) {
    swap_collide(shA, shB, ctx, result);
}

}
