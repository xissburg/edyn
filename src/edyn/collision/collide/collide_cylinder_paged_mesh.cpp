#include "edyn/collision/collide.hpp"
#include "edyn/shapes/triangle_shape.hpp"
#include "edyn/util/aabb_util.hpp"

namespace edyn {

void collide(const cylinder_shape &shA, const paged_mesh_shape &shB, 
             const collision_context &ctx, collision_result &result) {
    // Cylinder position and orientation in mesh's space.
    auto posA_in_B = rotate(conjugate(ctx.ornB), ctx.posA - ctx.posB);
    auto ornA_in_B = conjugate(ctx.ornB) * ctx.ornA;

    const auto cyl_axis = quaternion_x(ornA_in_B);
    const auto disc_center_pos = posA_in_B + cyl_axis * shA.half_length;
    const auto disc_center_neg = posA_in_B - cyl_axis * shA.half_length;

    auto aabb = shape_aabb(shA, posA_in_B, ornA_in_B);
    shB.trimesh->visit(aabb, [&] (size_t mesh_idx, size_t tri_idx, const triangle_vertices &vertices) {
        auto trimesh = shB.trimesh->get_submesh(mesh_idx);
        auto tri = triangle_shape{};
        tri.vertices = vertices;

        for (int i = 0; i < 3; ++i) {
            tri.is_concave_edge[i] = trimesh->is_concave_edge[tri_idx * 3 + i];
            tri.cos_angles[i] = trimesh->cos_angles[tri_idx * 3 + i];
        }

        tri.update_computed_properties();

        collide_cylinder_triangle(shA, posA_in_B, ornA_in_B, 
                                  disc_center_pos, disc_center_neg, cyl_axis, 
                                  tri, ctx.threshold, result);
    });
}

void collide(const paged_mesh_shape &shA, const cylinder_shape &shB,
             const collision_context &ctx, collision_result &result) {
    swap_collide(shA, shB, ctx, result);
}

}
