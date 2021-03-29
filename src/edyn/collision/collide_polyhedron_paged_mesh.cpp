#include "edyn/collision/collide.hpp"

namespace edyn {

collision_result collide(const polyhedron_shape &shA, const paged_mesh_shape &shB, 
                         const collision_context &ctx) {
    auto result = collision_result{};
    auto &rmeshA = *(*ctx.rmeshA);

    // Polyhedron position and orientation in mesh's space.
    const auto ornB_conj = conjugate(ctx.ornB);
    const auto posA_in_B = rotate(ornB_conj, ctx.posA - ctx.posB);
    const auto ornA_in_B = ornB_conj * ctx.ornA;

    auto aabb = shA.aabb(posA_in_B, ornA_in_B);
    shB.trimesh->visit(aabb, [&] (size_t mesh_idx, size_t tri_idx, const triangle_vertices &vertices) {
        std::array<vector3, 3> shifted_vertices;
        std::array<bool, 3> is_concave_edge;
        std::array<scalar, 3> cos_angles;
        auto trimesh = shB.trimesh->get_submesh(mesh_idx);

        for (int i = 0; i < 3; ++i) {
            shifted_vertices[i] = vertices[i] - ctx.posA;
            is_concave_edge[i] = trimesh->is_concave_edge[tri_idx * 3 + i];
            cos_angles[i] = trimesh->cos_angles[tri_idx * 3 + i];
        }

        collide_polyhedron_triangle(shA, rmeshA, shifted_vertices, 
                                    is_concave_edge, cos_angles, 
                                    ctx.threshold, result);
    });

    return result;
}

collision_result collide(const paged_mesh_shape &shA, const polyhedron_shape &shB,
                         const collision_context &ctx) {
    return swap_collide(shA, shB, ctx);
}

}
