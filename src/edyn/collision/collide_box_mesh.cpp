#include "edyn/collision/collide.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/math/math.hpp"

namespace edyn {

collision_result collide(const box_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const mesh_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold) {
    auto result = collision_result{};

    // Box position and orientation in mesh's space.
    const auto ornB_conj = conjugate(ornB);
    const auto posA_in_B = rotate(ornB_conj, posA - posB);
    const auto ornA_in_B = ornB_conj * ornA;

    const auto axesA = std::array<vector3, 3>{
        quaternion_x(ornA_in_B),
        quaternion_y(ornA_in_B),
        quaternion_z(ornA_in_B)
    };

    auto aabb = shA.aabb(posA_in_B, ornA_in_B);
    shB.trimesh->visit(aabb, [&] (size_t tri_idx, const triangle_vertices &vertices) {
        std::array<bool, 3> is_concave_edge;
        std::array<scalar, 3> cos_angles;

        for (int i = 0; i < 3; ++i) {
            is_concave_edge[i] = shB.trimesh->is_concave_edge[tri_idx * 3 + i];
            cos_angles[i] = shB.trimesh->cos_angles[tri_idx * 3 + i];
        }

        collide_box_triangle(shA, posA_in_B, ornA_in_B, axesA, vertices, 
                             is_concave_edge, cos_angles, threshold, result);
    });

    return result;
}

}
