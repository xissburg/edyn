#include "edyn/collision/collide.hpp"
#include "edyn/math/math.hpp"

namespace edyn {

collision_result collide(const box_shape &shA, const paged_mesh_shape &shB, 
                         const collision_context &ctx) {
    
    auto result = collision_result{};

    // Box position and orientation in mesh's space.
    const auto ornB_conj = conjugate(ctx.ornB);
    const auto posA_in_B = rotate(ornB_conj, ctx.posA - ctx.posB);
    const auto ornA_in_B = ornB_conj * ctx.ornA;

    const auto axesA = std::array<vector3, 3>{
        quaternion_x(ornA_in_B),
        quaternion_y(ornA_in_B),
        quaternion_z(ornA_in_B)
    };

    auto aabb = shA.aabb(posA_in_B, ornA_in_B);
    shB.trimesh->visit(aabb, [&] (size_t mesh_idx, size_t tri_idx, const triangle_vertices &vertices) {
        std::array<bool, 3> is_concave_edge;
        std::array<scalar, 3> cos_angles;
        auto trimesh = shB.trimesh->get_submesh(mesh_idx);

        for (int i = 0; i < 3; ++i) {
            is_concave_edge[i] = trimesh->is_concave_edge[tri_idx * 3 + i];
            cos_angles[i] = trimesh->cos_angles[tri_idx * 3 + i];
        }

        collide_box_triangle(shA, posA_in_B, ornA_in_B, axesA, vertices, 
                             is_concave_edge, cos_angles, ctx.threshold, result);
    });

    return result;
}

}
