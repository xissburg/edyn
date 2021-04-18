#include "edyn/collision/collide.hpp"
#include "edyn/util/aabb_util.hpp"

namespace edyn {

void collide(const compound_shape &shA, const compound_shape &shB,
             const collision_context &ctx, collision_result &result) {
    // Collide every node of B with A. Swap it if B has more nodes than A.
    if (shA.nodes.size() < shB.nodes.size()) {
        swap_collide(shA, shB, ctx, result);
        return;
    }

    for (auto &nodeB : shB.nodes) {
        // Create a new collision context with the position of the child of the
        // child of B in world space.
        auto child_ctx = ctx;
        child_ctx.posB = to_world_space(nodeB.position, ctx.posB, ctx.ornB);
        child_ctx.ornB = ctx.ornB * nodeB.orientation;
        child_ctx.aabbB = aabb_to_world_space(nodeB.aabb, ctx.posB, ctx.ornB);
        collision_result child_result;

        // Collide child shape with compound.
        std::visit([&] (auto &&sh) {
            collide(shA, sh, child_ctx, child_result);
        }, nodeB.var);

        // Transform the B elements of the result points from child shape space
        // into B's space.
        for (size_t i = 0; i < child_result.num_points; ++i) {
            auto &child_point = child_result.point[i];
            child_point.pivotB = to_world_space(child_point.pivotB, nodeB.position, nodeB.orientation);
            child_point.normalB = rotate(nodeB.orientation, child_point.normalB);
            result.maybe_add_point(child_point);
        }
    }
}

}