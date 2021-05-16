#include "edyn/collision/collide.hpp"
#include "edyn/shapes/triangle_shape.hpp"

namespace edyn {

void collide(const compound_shape &compound, const triangle_mesh &mesh,
             const collision_context &ctx, collision_result &result) {
    for (auto &node : compound.nodes) {
        // New collision context with child shape in world space.
        auto child_ctx = ctx;
        child_ctx.posA = to_world_space(node.position, ctx.posA, ctx.ornA);
        child_ctx.ornA = ctx.ornA * node.orientation;

        collision_result child_result;

        std::visit([&] (auto &&sh) {
            collide(sh, mesh, child_ctx, child_result);
        }, node.shape_var);

        // The elements of A in the collision points must be transformed from
        // the child node's space into the compound's space.
        for (size_t i = 0; i < child_result.num_points; ++i) {
            auto &child_point = child_result.point[i];
            child_point.pivotA = to_world_space(child_point.pivotA, node.position, node.orientation);
            result.maybe_add_point(child_point);
        }
    }
}

}
