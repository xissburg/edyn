#include "edyn/collision/collide.hpp"

namespace edyn {

collision_result collide(const polyhedron_shape &shA, const paged_mesh_shape &shB, 
                         const collision_context &ctx) {
    auto result = collision_result{};
    auto &rmeshA = ctx.rmeshA->get();

    // Polyhedron position and orientation in mesh's space.
    const auto ornB_conj = conjugate(ctx.ornB);
    const auto posA_in_B = rotate(ornB_conj, ctx.posA - ctx.posB);
    const auto ornA_in_B = ornB_conj * ctx.ornA;

    auto aabb = shA.aabb(posA_in_B, ornA_in_B);
    shB.trimesh->visit(aabb, [&] (size_t mesh_idx, size_t tri_idx, const triangle_vertices &vertices) {
        auto trimesh = shB.trimesh->get_submesh(mesh_idx);
        auto tri = triangle_shape{};

        for (int i = 0; i < 3; ++i) {
            tri.vertices[i] = vertices[i] - ctx.posA;
            tri.is_concave_edge[i] = trimesh->is_concave_edge[tri_idx * 3 + i];
            tri.cos_angles[i] = trimesh->cos_angles[tri_idx * 3 + i];
        }

        tri.update_computed_properties();

        collide_polyhedron_triangle(shA, rmeshA, ctx.posA, ctx.ornA, tri, ctx.threshold, result);
    });

    return result;
}

collision_result collide(const paged_mesh_shape &shA, const polyhedron_shape &shB,
                         const collision_context &ctx) {
    return swap_collide(shA, shB, ctx);
}

}
