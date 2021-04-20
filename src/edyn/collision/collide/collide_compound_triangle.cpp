#include "edyn/collision/collide.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/shapes/triangle_shape.hpp"

namespace edyn {

void collide(const compound_shape &compound, const triangle_shape &tri,
             const collision_context &ctx, collision_result &result) {
    auto local_tri = tri;
    local_tri.normal = rotate(conjugate(ctx.ornA), tri.normal);

    for (auto &vertex : local_tri.vertices) {
        vertex = to_object_space(vertex, ctx.posA, ctx.ornA);
    }

    auto aabb = get_triangle_aabb(local_tri.vertices);
    compound.visit(aabb, [&] (auto &&sh, const vector3 &pos, const quaternion &orn) {
        // New collision context set into A's space.
        auto child_ctx = ctx;
        child_ctx.posA = pos;
        child_ctx.ornA = orn;

        collision_result child_result;
        collide(sh, local_tri, child_ctx, child_result);

        // The elements of A in the collision points must be transformed from
        // the child node's space into A's space and the elements of B must be
        // transformed from A's space into world space since that's where the
        // original triangle comes from.
        for (size_t i = 0; i < child_result.num_points; ++i) {
            auto &child_point = child_result.point[i];
            child_point.pivotA = to_world_space(child_point.pivotA, pos, orn);
            child_point.pivotB = to_world_space(child_point.pivotB, ctx.posA, ctx.ornA);
            child_point.normalB = rotate(ctx.ornA, child_point.normalB);
            result.maybe_add_point(child_point);
        }
    });
}

}