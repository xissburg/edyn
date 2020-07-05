#include "edyn/collision/collide.hpp"
#include "edyn/shapes/triangle_shape.hpp"
#include "edyn/math/math.hpp"
#include <algorithm>

namespace edyn {

collision_result collide(const cylinder_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const paged_mesh_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold) {
    auto result = collision_result{};

    // Cylinder position and orientation in mesh's space.
    auto posA_in_B = rotate(conjugate(ornB), posA - posB);
    auto ornA_in_B = conjugate(ornB) * ornA;

    const auto cyl_axis = quaternion_x(ornA_in_B);
    const auto disc_center_pos = posA_in_B + cyl_axis * shA.half_length;
    const auto disc_center_neg = posA_in_B - cyl_axis * shA.half_length;

    auto aabb = shA.aabb(posA_in_B, ornA_in_B);
    shB.trimesh->visit(aabb, [&] (size_t mesh_idx, size_t tri_idx, const triangle_vertices &vertices) {
        std::array<bool, 3> is_concave_edge;
        std::array<scalar, 3> cos_angles;
        auto *trimesh = shB.trimesh->get_submesh(mesh_idx);

        for (int i = 0; i < 3; ++i) {
            is_concave_edge[i] = trimesh->is_concave_edge[tri_idx * 3 + i];
            cos_angles[i] = trimesh->cos_angles[tri_idx * 3 + i];
        }

        collide_cylinder_triangle(shA, posA_in_B, ornA_in_B, 
                                  disc_center_pos, disc_center_neg, cyl_axis, vertices, 
                                  is_concave_edge, cos_angles, threshold, result);
    });

    return result;
}

}