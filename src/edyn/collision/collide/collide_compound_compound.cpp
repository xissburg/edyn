#include "edyn/collision/collide.hpp"
#include "edyn/math/transform.hpp"

namespace edyn {

void collide(const compound_shape &shA, const compound_shape &shB,
             const collision_context &ctx, collision_result &result) {
    // Collide every node of B with A. Swap it if B has more nodes than A.
    if (shA.nodes.size() < shB.nodes.size()) {
        swap_collide(shA, shB, ctx, result);
        return;
    }

    for (size_t node_idx = 0; node_idx < shB.nodes.size(); ++node_idx) {
        auto &nodeB = shB.nodes[node_idx];

        // Create a new collision context with the position of the child of B
        // in world space.
        auto child_ctx = ctx;
        child_ctx.posB = to_world_space(nodeB.position, ctx.posB, ctx.ornB);
        child_ctx.ornB = ctx.ornB * nodeB.orientation;
        collision_result child_result;

        // Collide child shape with compound.
        std::visit([&](auto &&sh) {
            collide(shA, sh, child_ctx, child_result);
        }, nodeB.shape_var);

        // Transform the B elements of the result points from child shape space
        // into B's space.
        for (size_t i = 0; i < child_result.num_points; ++i) {
            auto &child_point = child_result.point[i];
            child_point.pivotB = to_world_space(child_point.pivotB, nodeB.position, nodeB.orientation);

            // Assign the part index for the second compound. The part index
            // for featureA was already assigned in the call to `collide` above.
            if (!child_point.featureB) {
                child_point.featureB = collision_feature{};
            }

            child_point.featureB->part = node_idx;

            result.maybe_add_point(child_point);
        }
    }
}

}
