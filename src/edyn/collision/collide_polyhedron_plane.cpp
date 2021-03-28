#include "edyn/collision/collide.hpp"
#include "edyn/collision/collision_result.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/math/scalar.hpp"

namespace edyn {

collision_result collide(const polyhedron_shape &shA, const plane_shape &shB, 
                         const collision_context &ctx) {
    const auto &posA = ctx.posA;
    const auto &ornA = ctx.ornA;
    const auto &posB = ctx.posB;
    const auto &ornB = ctx.ornB;

    auto result = collision_result{};
    auto normal = rotate(ornB, shB.normal);
    auto center = posB + rotate(ornB, shB.normal * shB.constant);
    scalar min_distance = EDYN_SCALAR_MAX;

    // Find distance between polyhedron and plane.
    for (size_t i = 0; i < shA.mesh->vertices.size(); ++i) {
        auto vertex_local = shA.mesh->vertices[i];
        auto vertex_world = to_world_space(vertex_local, posA, ornA);
        auto distance = dot(vertex_world - center, normal);

        if (distance < min_distance) {
            min_distance = distance;
        }
    }

    if (min_distance > ctx.threshold) return {};

    // Add points to all vertices that are within a range from the
    // minimum distance.
    const scalar distance_range = 0.002;

    for (size_t i = 0; i < shA.mesh->vertices.size(); ++i) {
        auto vertex_local = shA.mesh->vertices[i];
        auto vertex_world = to_world_space(vertex_local, posA, ornA);
        auto distance = dot(vertex_world - center, normal);

        if (distance > min_distance + distance_range) continue;

        auto pt_proj = vertex_world - normal * distance;
        auto pivotB = rotate(conjugate(ornB), pt_proj - posB);
        result.maybe_add_point({vertex_local, pivotB, shB.normal, distance});
    }

    return result;
}

}
