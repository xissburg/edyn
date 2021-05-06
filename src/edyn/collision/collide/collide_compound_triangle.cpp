#include "edyn/collision/collide.hpp"
#include "edyn/shapes/triangle_shape.hpp"

namespace edyn {

void collide(const compound_shape &compound, const triangle_shape &tri,
             const collision_context &ctx, collision_result &result) {
    auto local_vertices = tri.vertices;

    for (auto &vertex : local_vertices) {
        vertex = to_object_space(vertex, ctx.posA, ctx.ornA);
    }

    auto aabb = get_triangle_aabb(local_vertices);

    compound.visit(aabb, [&] (auto &&sh, const compound_shape::shape_node &node) {
        // New collision context with child shape in world space.
        auto child_ctx = ctx;
        child_ctx.posA = to_world_space(node.position, ctx.posA, ctx.ornA);
        child_ctx.ornA = ctx.ornA * node.orientation;

        collision_result child_result;
        collide(sh, tri, child_ctx, child_result);

        // The elements of A in the collision points must be transformed from
        // the child node's space into the compound's space.
        for (size_t i = 0; i < child_result.num_points; ++i) {
            auto &child_point = child_result.point[i];
            child_point.pivotA = to_world_space(child_point.pivotA, node.position, node.orientation);
            result.maybe_add_point(child_point);
        }
    });
}

}
