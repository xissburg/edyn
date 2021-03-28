#include "edyn/collision/collide.hpp"

namespace edyn {

collision_result collide(const sphere_shape &shA, const mesh_shape &shB, 
                         const collision_context &ctx) {
    auto result = collision_result{};

    // Sphere position in mesh's space.
    auto posA_in_B = rotate(conjugate(ctx.ornB), ctx.posA - ctx.posB);
    auto ornA_in_B = conjugate(ctx.ornB) * ctx.ornA;

    auto aabb = shA.aabb(posA_in_B, ctx.ornA); // Invariant to orientation.
    shB.trimesh->visit(aabb, [&] (size_t tri_idx, const triangle_vertices &vertices) {
        if (result.num_points == max_contacts) {
            return;
        }

        std::array<bool, 3> is_concave_edge;
        std::array<scalar, 3> cos_angles;

        for (int i = 0; i < 3; ++i) {
            is_concave_edge[i] = shB.trimesh->is_concave_edge[tri_idx * 3 + i];
            cos_angles[i] = shB.trimesh->cos_angles[tri_idx * 3 + i];
        }

        collide_sphere_triangle(shA, posA_in_B, ornA_in_B, vertices, 
                                is_concave_edge, cos_angles, ctx.threshold, result);
    });

    return result;
}

}
