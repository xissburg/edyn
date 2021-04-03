#include "edyn/collision/collide.hpp"
#include "edyn/shapes/triangle_shape.hpp"
#include "edyn/math/math.hpp"
#include <algorithm>

namespace edyn {

collision_result collide(const cylinder_shape &shA, const mesh_shape &shB, 
                         const collision_context &ctx) {
    auto result = collision_result{};

    // Cylinder position and orientation in mesh's space.
    auto posA_in_B = rotate(conjugate(ctx.ornB), ctx.posA - ctx.posB);
    auto ornA_in_B = conjugate(ctx.ornB) * ctx.ornA;

    const auto cyl_axis = quaternion_x(ornA_in_B);
    const auto disc_center_pos = posA_in_B + cyl_axis * shA.half_length;
    const auto disc_center_neg = posA_in_B - cyl_axis * shA.half_length;

    auto aabb = shA.aabb(posA_in_B, ornA_in_B);
    shB.trimesh->visit(aabb, [&] (size_t tri_idx, const triangle_vertices &vertices) {
        auto tri = triangle_shape{};
        tri.vertices = vertices;

        for (int i = 0; i < 3; ++i) {
            tri.is_concave_edge[i] = shB.trimesh->is_concave_edge[tri_idx * 3 + i];
            tri.cos_angles[i] = shB.trimesh->cos_angles[tri_idx * 3 + i];
        }

        collide_cylinder_triangle(shA, posA_in_B, ornA_in_B, 
                                  disc_center_pos, disc_center_neg, cyl_axis, 
                                  tri, ctx.threshold, result);
    });

    return result;
}

collision_result collide(const mesh_shape &shA, const cylinder_shape &shB,
                         const collision_context &ctx) {
    return swap_collide(shA, shB, ctx);
}

}
