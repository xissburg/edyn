#include "edyn/collision/collide.hpp"
#include "edyn/collision/collision_result.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/math/scalar.hpp"

namespace edyn {

void collide(const polyhedron_shape &shA, const plane_shape &shB, 
             const collision_context &ctx, collision_result &result) {
    const auto &posA = ctx.posA;
    const auto &posB = ctx.posB;
    const auto &ornB = ctx.ornB;

    auto &rmeshA = ctx.rmeshA->get();

    auto normal = rotate(ornB, shB.normal);
    auto center = posB + rotate(ornB, shB.normal * shB.constant);
    scalar distance = EDYN_SCALAR_MAX;

    // Find distance between polyhedron and plane.
    for (auto &rvertex : rmeshA.vertices) {
        auto vertex_world = posA + rvertex;
        auto vertex_dist = dot(vertex_world - center, normal);
        distance = std::min(vertex_dist, distance);
    }

    if (distance > ctx.threshold) return;

    // Add points to all vertices that are within a range from the
    // minimum distance.
    for (size_t i = 0; i < rmeshA.vertices.size(); ++i) {
        auto vertex_world = posA + rmeshA.vertices[i];
        auto vertex_dist = dot(vertex_world - center, normal);

        if (vertex_dist > distance + support_polygon_tolerance) continue;

        auto pivotA = shA.mesh->vertices[i];
        auto pivotB_world = vertex_world - normal * vertex_dist;
        auto pivotB = to_object_space(pivotB_world, posB, ornB);
        result.maybe_add_point({pivotA, pivotB, shB.normal, vertex_dist});
    }
}

void collide(const plane_shape &shA, const polyhedron_shape &shB,
             const collision_context &ctx, collision_result &result) {
    swap_collide(shA, shB, ctx, result);
}

}
