#include "edyn/collision/collide.hpp"
#include "edyn/collision/collision_result.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/math/scalar.hpp"

namespace edyn {

collision_result collide(const polyhedron_shape &shA, const plane_shape &shB, 
                         const collision_context &ctx) {
    const auto &posA = ctx.posA;
    const auto &posB = ctx.posB;
    const auto &ornB = ctx.ornB;

    auto &rmeshA = ctx.rmeshA->get();

    auto normal = rotate(ornB, shB.normal);
    auto center = posB + rotate(ornB, shB.normal * shB.constant);
    scalar min_distance = EDYN_SCALAR_MAX;

    // Find distance between polyhedron and plane.
    for (auto &rvertex : rmeshA.vertices) {
        auto vertex_world = posA + rvertex;
        auto distance = dot(vertex_world - center, normal);

        if (distance < min_distance) {
            min_distance = distance;
        }
    }

    if (min_distance > ctx.threshold) return {};

    // Add points to all vertices that are within a range from the
    // minimum distance.
    const scalar distance_range = 0.002;
    auto result = collision_result{};

    for (size_t i = 0; i < rmeshA.vertices.size(); ++i) {
        auto vertex_world = posA + rmeshA.vertices[i];
        auto distance = dot(vertex_world - center, normal);

        if (distance > min_distance + distance_range) continue;

        auto pivotA = shA.mesh->vertices[i];
        auto pt_proj = vertex_world - normal * distance;
        auto pivotB = to_object_space(pt_proj, posB, ornB);
        result.maybe_add_point({pivotA, pivotB, shB.normal, distance});
    }

    return result;
}

collision_result collide(const plane_shape &shA, const polyhedron_shape &shB,
                         const collision_context &ctx) {
    return swap_collide(shA, shB, ctx);
}

}
