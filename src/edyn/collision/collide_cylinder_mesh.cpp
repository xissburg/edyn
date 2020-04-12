#include "edyn/collision/collide.hpp"

namespace edyn {

struct separating_axis {
    vector3 pos;
    vector3 dir;
    scalar minA, maxA;
    scalar minB, maxB;
};

collision_result collide(const cylinder_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const mesh_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold) {
    auto result = collision_result{};

    // Cylinder position in mesh's space.
    auto posA_in_B = rotate(conjugate(ornB), posA - posB);
    auto ornA_in_B = conjugate(ornB) * ornA;
    auto aabb = shA.aabb(posA_in_B, ornA_in_B);
    shB.trimesh->visit(aabb, [&] (size_t tri_idx, const triangle_vertices &vertices) {
        if (result.num_points == max_contacts) {
            return;
        }

        std::vector<separating_axis> sep_axes;

        // Cylinder cap normal.
        {
            auto &axis = sep_axes.emplace_back();
            axis.dir = quaternion_x(ornA_in_B);
            axis.pos = posA_in_B;
            axis.minA = -shA.half_length;
            axis.maxA = shA.half_length;

            axis.minA = EDYN_SCALAR_MAX;
            axis.maxA = -EDYN_SCALAR_MAX;

            for (auto &v : vertices) {
                auto proj0 = dot(v - axis.pos, axis.dir);
                if (proj0 > axis.maxA) {
                    axis.maxA = proj0;
                }
                
                auto proj1 = dot(v - axis.pos, -axis.dir);
                if (proj1 < axis.minA) {
                    axis.minA = proj1;
                }
            }


        }

        // Face normal.
        {
            auto &axis = sep_axes.emplace_back();

            auto edges = get_triangle_edges(vertices);
            axis.dir = normalize(cross(edges[0], edges[1]));
            axis.pos = vertices[0];
            axis.minB = 0;
            axis.maxB = 0;
            
            auto p0 = shA.support_point(posA_in_B, ornA_in_B, -axis.dir);
            auto p1 = shA.support_point(posA_in_B, ornA_in_B, axis.dir);
            axis.minA = dot(p0 - axis.pos, axis.dir);
            axis.maxA = dot(p1 - axis.pos, axis.dir);
        }

        auto penetration = EDYN_SCALAR_MAX;
        size_t sep_axis_idx;

        for (size_t i = 0; i < sep_axes.size(); ++i) {
            auto &axis = sep_axes[i];

            if ((axis.minA < axis.minB && axis.maxA < axis.minB) ||
                (axis.minA > axis.maxB && axis.maxA > axis.maxB)) {
                continue;
            }

            scalar overlap_min, overlap_max;

            if (axis.minA < axis.minB && axis.minB < axis.maxA) {
                overlap_min = axis.minB;

                if (axis.maxA < axis.maxB) {
                    overlap_max = axis.maxA;
                } else {
                    overlap_max = axis.maxB;
                }
            } else if (axis.maxB < axis.maxA) {
                overlap_min = axis.minA;
                overlap_max = axis.maxB;
            } else {
                overlap_min = axis.minA;
                overlap_max = axis.maxA;
            }

            auto overlap = overlap_max - overlap_min;

            if (overlap < penetration) {
                penetration = overlap;
                sep_axis_idx = i;
            }
        }

        if (penetration < threshold) {
            auto &axis = sep_axes[sep_axis_idx];
            auto idx = result.num_points++;
            result.point[idx].pivotA = axis.pos + axis.dir * overlap_min;
            result.point[idx].pivotB = axis.pos + axis.dir * overlap_max;
            result.point[idx].normalB = axis.dir;
            result.point[idx].distance = penetration;
        }
    });

    return result;
}

}