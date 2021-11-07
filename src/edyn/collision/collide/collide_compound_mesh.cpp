#include "edyn/collision/collide.hpp"
#include "edyn/util/aabb_util.hpp"
#include "edyn/util/triangle_util.hpp"
#include "edyn/math/transform.hpp"

namespace edyn {

void collide(const compound_shape &compound, const triangle_mesh &mesh,
             const collision_context &ctx, collision_result &result) {
    // TODO Possible optimization: find the triangle mesh node which encompasses
    // the compound's AABB and start the tree queries from that node in the
    // child collision tests.

    for (size_t node_idx = 0; node_idx < compound.nodes.size(); ++node_idx) {
        auto &node = compound.nodes[node_idx];

        // New collision context with child shape in world space.
        auto child_ctx = ctx;
        child_ctx.posA = to_world_space(node.position, ctx.posA, ctx.ornA);
        child_ctx.ornA = ctx.ornA * node.orientation;
        child_ctx.aabbA = aabb_to_world_space(node.aabb, ctx.posA, ctx.ornA);

        collision_result child_result;

        std::visit([&] (auto &&sh) {
            collide(sh, mesh, child_ctx, child_result);
        }, node.shape_var);

        // The elements of A in the collision points must be transformed from
        // the child node's space into the compound's space.
        for (size_t i = 0; i < child_result.num_points; ++i) {
            auto &child_point = child_result.point[i];
            child_point.pivotA = to_world_space(child_point.pivotA, node.position, node.orientation);

            // Assign part index for the closest feature in the compound shape.
            if (!child_point.featureA) {
                child_point.featureA = {};
            }

            child_point.featureA->part = node_idx;

            result.maybe_add_point(child_point);
        }
    }
}

}
