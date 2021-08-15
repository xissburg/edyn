#include "edyn/collision/collide.hpp"
#include "edyn/math/geom.hpp"

namespace edyn {

void collide(const capsule_shape &shA, const capsule_shape &shB,
             const collision_context &ctx, collision_result &result) {
    const auto &posA = ctx.posA;
    const auto &ornA = ctx.ornA;
    const auto &posB = ctx.posB;
    const auto &ornB = ctx.ornB;

    auto verticesA = shA.get_vertices(posA, ornA);
    auto verticesB = shB.get_vertices(posB, ornB);

    scalar s[2], t[2];
    vector3 closestA[2], closestB[2];
    size_t num_points;

    auto dist_sqr = closest_point_segment_segment(verticesA[0], verticesA[1],
                                                  verticesB[0], verticesB[1],
                                                  s[0], t[0], closestA[0], closestB[0], &num_points,
                                                  &s[1], &t[1], &closestA[1], &closestB[1]);
    auto min_dist = shA.radius + shB.radius + ctx.threshold;

    if (dist_sqr > min_dist * min_dist) {
        return;
    }

    vector3 normal;
    scalar distance;

    if (dist_sqr > EDYN_EPSILON) {
        auto dist = std::sqrt(dist_sqr);
        normal = (closestA[0] - closestB[0]) / dist;
        distance = dist - shA.radius - shB.radius;
    } else {
        // Seguments intersect in 3D.
        auto axisA = verticesA[1] - verticesA[0];
        auto axisB = verticesB[1] - verticesB[0];
        normal = cross(axisA, axisB);

        if (dot(posA - posB, normal) < 0) {
            normal *= -1; // Make it point towards A.
        }

        if (!try_normalize(normal)) {
            normal = vector3_y;
        }

        distance = -(shA.radius + shB.radius);
    }

    for (size_t i = 0; i < num_points; ++i) {
        auto pivotA_world = closestA[i] - normal * shA.radius;
        auto pivotB_world = closestB[i] + normal * shB.radius;
        auto pivotA = to_object_space(pivotA_world, posA, ornA);
        auto pivotB = to_object_space(pivotB_world, posB, ornB);
        result.add_point({pivotA, pivotB, normal, distance, contact_normal_attachment::none});
    }
}

}
