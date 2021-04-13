#include "edyn/collision/collide.hpp"
#include "edyn/util/aabb_util.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/math/math.hpp"

namespace edyn {

collision_result collide(const box_shape &shA, const mesh_shape &shB, 
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

    auto aabb = shape_aabb(shA, posA_in_B, ornA_in_B);
    shB.trimesh->visit(aabb, [&] (size_t tri_idx, const triangle_vertices &vertices) {
        auto tri = triangle_shape{};
        tri.vertices = vertices;

        for (int i = 0; i < 3; ++i) {
            tri.is_concave_edge[i] = shB.trimesh->is_concave_edge[tri_idx * 3 + i];
            tri.cos_angles[i] = shB.trimesh->cos_angles[tri_idx * 3 + i];
        }

        tri.update_computed_properties();

        collide_box_triangle(shA, posA_in_B, ornA_in_B, axesA, tri, ctx.threshold, result);
    });

    return result;
}

collision_result collide(const mesh_shape &shA, const box_shape &shB,
                         const collision_context &ctx) {
    return swap_collide(shA, shB, ctx);
}

}
