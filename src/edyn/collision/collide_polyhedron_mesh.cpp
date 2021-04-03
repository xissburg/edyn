#include "edyn/collision/collide.hpp"

namespace edyn {

collision_result collide(const polyhedron_shape &shA, const mesh_shape &shB, 
                         const collision_context &ctx) {
    auto result = collision_result{};
    auto &rmeshA = *(*ctx.rmeshA);

    // Polyhedron's position and orientation in mesh's space.
    const auto ornB_conj = conjugate(ctx.ornB);
    const auto posA_in_B = rotate(ornB_conj, ctx.posA - ctx.posB);
    const auto ornA_in_B = ornB_conj * ctx.ornA;

    auto aabb = shA.aabb(posA_in_B, ornA_in_B);
    shB.trimesh->visit(aabb, [&] (size_t tri_idx, const triangle_vertices &vertices) {
        auto tri = triangle_shape{};

        for (int i = 0; i < 3; ++i) {
            tri.vertices[i] = vertices[i] - ctx.posA;
            tri.is_concave_edge[i] = shB.trimesh->is_concave_edge[tri_idx * 3 + i];
            tri.cos_angles[i] = shB.trimesh->cos_angles[tri_idx * 3 + i];
        }

        collide_polyhedron_triangle(shA, rmeshA, tri, ctx.threshold, result);
    });

    return result;
}

collision_result collide(const mesh_shape &shA, const polyhedron_shape &shB,
                         const collision_context &ctx) {
    return swap_collide(shA, shB, ctx);
}

}
