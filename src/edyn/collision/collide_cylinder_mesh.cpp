#include "edyn/collision/collide.hpp"
#include "edyn/shapes/triangle_shape.hpp"
#include "edyn/math/math.hpp"
#include <algorithm>

namespace edyn {

collision_result collide(const cylinder_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const mesh_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold) {
    auto result = collision_result{};

    // Cylinder position and orientation in mesh's space.
    auto posA_in_B = rotate(conjugate(ornB), posA - posB);
    auto ornA_in_B = conjugate(ornB) * ornA;

    const auto cyl_axis = quaternion_x(ornA_in_B);
    const auto disc_center_pos = posA_in_B + cyl_axis * shA.half_length;
    const auto disc_center_neg = posA_in_B - cyl_axis * shA.half_length;

    auto aabb = shA.aabb(posA_in_B, ornA_in_B);
    shB.trimesh->visit(aabb, [&] (size_t tri_idx, const triangle_vertices &vertices) {
        // Separating-axis test. Find axis with greatest distance between projection
        // intervals.
        // Axes to be tested:
        // - Cylinder cap normals. Simply find the triangle vertices that are 
        //   further down in both directions.
        // - Triangle face normal. The cylinder could be laying sideways onto the
        //   triangle face, or it could be erect above the triangle face thus having
        //   one of its caps sitting on it, or else the axis projections can be 
        //   found via the support points of the caps along the negative triangle
        //   normal.
        // - Cylinder sidewall faces and the cross product between sidewall edges and
        //   triangle edges. The sidewalls are thought to have infinitely thin faces 
        //   and edges running straight from one cap to the other. They're handled 
        //   together using the cross product between the cylinder axis and each
        //   triangle axis as a separating axis.
        // - Cylinder face edges against triangle edges. The closest point between
        //   the circle and edge segment are calculated. The vector connecting them
        //   is taken as the separating axis. The projections must then be calculated
        //   using support points. 
        std::array<bool, 3> is_concave_edge;
        std::array<scalar, 3> cos_angles;

        for (int i = 0; i < 3; ++i) {
            is_concave_edge[i] = shB.trimesh->is_concave_edge[tri_idx * 3 + i];
            cos_angles[i] = shB.trimesh->cos_angles[tri_idx * 3 + i];
        }

        collide_cylinder_triangle(shA, posA_in_B, ornA_in_B, 
                                  disc_center_pos, disc_center_neg, cyl_axis, vertices, 
                                  is_concave_edge, cos_angles, threshold, result);
    });

    return result;
}

}