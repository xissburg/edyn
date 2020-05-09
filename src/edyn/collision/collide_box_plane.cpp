#include "edyn/collision/collide.hpp"

namespace edyn {

collision_result collide(const box_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const plane_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold) {
    auto result = collision_result{};
    auto normal = rotate(ornB, shB.normal);
    auto center = posB + rotate(ornB, shB.normal * shB.constant);

    for (size_t i = 0; i < 8; ++i) {
        auto vertex_local = shA.get_vertex(i);
        auto vertex_world = posA + rotate(ornA, vertex_local);
        auto distance = dot(vertex_world - center, normal);

        if (distance < threshold) {
            auto pt_proj = vertex_world - normal * distance;
            auto idx = SIZE_MAX;

            if (result.num_points == max_contacts) {
                // If the results are full and this contact is deeper than the least
                // deep contact, replace it.
                auto max_dist = -EDYN_SCALAR_MAX;
                size_t max_dist_idx;

                for (size_t j = 0; j < result.num_points; ++j) {
                    auto dist = result.point[j].distance;
                    if (dist > max_dist) {
                        max_dist_idx = j;
                        max_dist = dist;
                    }
                }

                if (distance < result.point[max_dist_idx].distance) {
                    idx = max_dist_idx;
                }
            } else {
                idx = result.num_points++;
            }

            if (idx != SIZE_MAX) {
                result.point[idx].pivotA = vertex_local;
                result.point[idx].pivotB = rotate(conjugate(ornB), pt_proj - posB);
                result.point[idx].normalB = shB.normal;
                result.point[idx].distance = distance;
            }
        }
    }

    return result;
}

}