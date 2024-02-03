#include "edyn/collision/collide.hpp"
#include "edyn/collision/collision_result.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/math/transform.hpp"
#include "edyn/util/shape_util.hpp"

namespace edyn {

void collide(const polyhedron_shape &shA, const plane_shape &shB,
             const collision_context &ctx, collision_result &result) {
    auto &posA = ctx.posA;
    auto &rmeshA = *shA.rotated;

    auto normal = shB.normal;
    auto center = shB.normal * shB.constant - posA;

    auto proj_poly = -polyhedron_support_projection(rmeshA.vertices, shA.mesh->neighbors_start, shA.mesh->neighbor_indices, -normal);
    auto proj_plane = dot(center, normal);
    scalar distance = proj_poly - proj_plane;

    if (distance > ctx.threshold) return;

    auto polygon = point_cloud_support_polygon(
        rmeshA.vertices.begin(), rmeshA.vertices.end(), vector3_zero,
        normal, proj_poly, true, support_feature_tolerance);
    auto normal_attachment = contact_normal_attachment::normal_on_B;

    for (auto idxA : polygon.hull) {
        auto &pointA = polygon.vertices[idxA];

        auto pivotA = rotate(conjugate(ctx.ornA), pointA);
        auto local_distance = dot(pointA - center, normal);
        auto pivotB = pointA - normal * local_distance + posA; // Project onto plane.
        result.maybe_add_point({pivotA, pivotB, normal, local_distance, normal_attachment});
    }
}

void collide(const plane_shape &shA, const polyhedron_shape &shB,
             const collision_context &ctx, collision_result &result) {
    swap_collide(shA, shB, ctx, result);
}

}
