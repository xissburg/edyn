#include "edyn/collision/collide.hpp"
#include "edyn/shapes/triangle_shape.hpp"
#include "edyn/util/aabb_util.hpp"

namespace edyn {

void collide(const capsule_shape &shA, const mesh_shape &shB, 
             const collision_context &ctx, collision_result &result) {
    // Capsule position and orientation in mesh's space.
    auto posA_in_B = rotate(conjugate(ctx.ornB), ctx.posA - ctx.posB);
    auto ornA_in_B = conjugate(ctx.ornB) * ctx.ornA;
    auto capsule_vertices = shA.get_vertices(posA_in_B, ornA_in_B);

    auto aabb = shape_aabb(shA, posA_in_B, ornA_in_B);
    shB.trimesh->visit(aabb, [&] (size_t tri_idx, const triangle_vertices &vertices) {
        auto tri = triangle_shape{};
        tri.vertices = vertices;

        for (int i = 0; i < 3; ++i) {
            tri.is_concave_edge[i] = shB.trimesh->is_concave_edge[tri_idx * 3 + i];
            tri.cos_angles[i] = shB.trimesh->cos_angles[tri_idx * 3 + i];
        }

        tri.update_computed_properties();

        collide_capsule_triangle(shA, posA_in_B, ornA_in_B, capsule_vertices, 
                                 tri, ctx.threshold, result);
    });
}

void collide(const mesh_shape &shA, const capsule_shape &shB,
             const collision_context &ctx, collision_result &result) {
    swap_collide(shA, shB, ctx, result);
}

}
