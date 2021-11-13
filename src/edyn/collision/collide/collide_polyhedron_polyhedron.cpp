#include "edyn/collision/collide.hpp"
#include "edyn/math/math.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/math/vector2_3_util.hpp"
#include "edyn/math/transform.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/util/shape_util.hpp"

namespace edyn {

// Finds the direction that maximizes the projected distance between
// A and B among all face normals of A.
static
void max_support_direction(const polyhedron_shape &shA, const rotated_mesh &rotatedA, const vector3 &posA,
                           const polyhedron_shape &shB, const rotated_mesh &rotatedB, const vector3 &posB,
                           vector3 &dir, scalar &distance, scalar &projectionA, scalar &projectionB) {
    scalar max_proj_A = EDYN_SCALAR_MAX;
    scalar max_proj_B = -EDYN_SCALAR_MAX;
    scalar max_distance = -EDYN_SCALAR_MAX;
    auto best_dir = vector3_zero;

    for (size_t i = 0; i < rotatedA.relevant_normals.size(); ++i) {
        auto normal_world = -rotatedA.relevant_normals[i]; // Normal pointing towards A.
        auto vertexA = rotatedA.vertices[shA.mesh->relevant_indices[i]];
        auto vertex_world = vertexA + posA;
        auto projA = dot(vertex_world, normal_world);

        // Find point on B that's furthest along the opposite direction
        // of the face normal.
        auto projB = point_cloud_support_projection(rotatedB.vertices, normal_world) + dot(posB, normal_world);

        auto dist = projA - projB;

        if (dist > max_distance) {
            max_distance = dist;
            max_proj_A = projA;
            max_proj_B = projB;
            best_dir = normal_world;
        }
    }

    dir = best_dir;
    distance = max_distance;
    projectionA = max_proj_A;
    projectionB = max_proj_B;
}

void collide(const polyhedron_shape &shA, const polyhedron_shape &shB,
             const collision_context &ctx, collision_result &result) {
    // Calculate collision with shape A in the origin for better floating point
    // precision. Position of shape B is modified accordingly.
    const auto posA = vector3_zero;
    const auto &ornA = ctx.ornA;
    const auto posB = ctx.posB - ctx.posA;
    const auto &ornB = ctx.ornB;
    const auto threshold = ctx.threshold;

    // The pre-rotated vertices and normals are used to avoid rotating vertices
    // every time.
    auto &rmeshA = *shA.rotated;
    auto &rmeshB = *shB.rotated;

    scalar distance = -EDYN_SCALAR_MAX;
    scalar projectionA = EDYN_SCALAR_MAX;
    scalar projectionB = -EDYN_SCALAR_MAX;
    auto sep_axis = vector3_zero;

    // Find best support direction among all face normals of A.
    max_support_direction(shA, rmeshA, posA, shB, rmeshB, posB,
                          sep_axis, distance, projectionA, projectionB);

    // Find best support direction among all face normals of B.
    {
        scalar dist, projA, projB;
        vector3 dir;
        max_support_direction(shB, rmeshB, posB, shA, rmeshA, posA,
                              dir, dist, projB, projA);

        if (dist > distance) {
            // Signs must be flipped because parameters were swapped above.
            dir *= -1;
            projA *= -1;
            projB *= -1;

            distance = dist;
            projectionA = projA;
            projectionB = projB;
            sep_axis = dir;
        }
    }

    // Edge vs edge.
    for (auto &edgeA : rmeshA.relevant_edges) {
        for (auto &edgeB : rmeshB.relevant_edges) {
            auto dir = cross(edgeA, edgeB);

            if (!try_normalize(dir)) {
                continue;
            }

            if (dot(posA - posB, dir) < 0) {
                // Make it point towards A.
                dir *= -1;
            }

            auto projA = -point_cloud_support_projection(rmeshA.vertices, -dir);
            auto projB = point_cloud_support_projection(rmeshB.vertices, dir) + dot(posB, dir);
            auto dist = projA - projB;

            if (dist > distance) {
                distance = dist;
                projectionA = projA;
                projectionB = projB;
                sep_axis = dir;
            }
        }
    }

    if (distance > threshold) {
        return;
    }

    auto polygonA = point_cloud_support_polygon(
        rmeshA.vertices.begin(), rmeshA.vertices.end(), vector3_zero,
        sep_axis, projectionA, true, support_feature_tolerance);
    auto polygonB = point_cloud_support_polygon(
        rmeshB.vertices.begin(), rmeshB.vertices.end(), posB,
        sep_axis, projectionB, false, support_feature_tolerance);

    auto normal_attachment = contact_normal_attachment::none;

    if (polygonB.hull.size() > 2) {
        normal_attachment = contact_normal_attachment::normal_on_B;
    } else if (polygonA.hull.size() > 2) {
        normal_attachment = contact_normal_attachment::normal_on_A;
    }

    // First, add contact points for vertices that lie inside the opposing face.
    // If the feature on B is a face, i.e. `verticesB` has 3 or more elements,
    // check if the points in `verticesA` lie inside the prism spanned by `verticesB`
    // and `sep_axis`
    if (polygonB.hull.size() > 2) {
        for (auto idxA : polygonA.hull) {
            auto &pointA = polygonA.vertices[idxA];

            if (point_in_polygonal_prism(polygonB.vertices, polygonB.hull, sep_axis, pointA)) {
                auto pivotA = to_object_space(pointA, posA, ornA);
                auto pivotB_world = project_plane(pointA, polygonB.origin, sep_axis);
                auto pivotB = to_object_space(pivotB_world, posB, ornB);
                result.maybe_add_point({pivotA, pivotB, sep_axis, distance, normal_attachment});
            }
        }
    }

    if (polygonA.hull.size() > 2) {
        for (auto idxB : polygonB.hull) {
            auto &pointB = polygonB.vertices[idxB];

            if (point_in_polygonal_prism(polygonA.vertices, polygonA.hull, sep_axis, pointB)) {
                auto pivotB = to_object_space(pointB, posB, ornB);
                auto pivotA_world = project_plane(pointB, polygonA.origin, sep_axis);
                auto pivotA = to_object_space(pivotA_world, posA, ornA);
                result.maybe_add_point({pivotA, pivotB, sep_axis, distance, normal_attachment});
            }
        }
    }

    // Calculate 2D intersection of edges on the closest features.
    if (polygonA.hull.size() > 1 && polygonB.hull.size() > 1) {
        // If the feature is a polygon, it will be necessary to wrap around the
        // vertex array. If it is just one edge, then avoid calculating the same
        // segment-segment intersection twice.
        const auto sizeA = polygonA.hull.size();
        const auto sizeB = polygonB.hull.size();
        const auto limitA = sizeA == 2 ? 1 : sizeA;
        const auto limitB = sizeB == 2 ? 1 : sizeB;
        scalar s[2], t[2];

        for (size_t i = 0; i < limitA; ++i) {
            auto idx0A = polygonA.hull[i];
            auto idx1A = polygonA.hull[(i + 1) % sizeA];
            auto &v0A = polygonA.plane_vertices[idx0A];
            auto &v1A = polygonA.plane_vertices[idx1A];

            for (size_t j = 0; j < limitB; ++j) {
                auto idx0B = polygonB.hull[j];
                auto idx1B = polygonB.hull[(j + 1) % sizeB];
                auto &v0B = polygonB.plane_vertices[idx0B];
                auto &v1B = polygonB.plane_vertices[idx1B];
                auto num_points = intersect_segments(v0A, v1A, v0B, v1B,
                                                     s[0], t[0], s[1], t[1]);

                for (size_t k = 0; k < num_points; ++k) {
                    auto pivotA_world = lerp(polygonA.vertices[idx0A], polygonA.vertices[idx1A], s[k]);
                    auto pivotB_world = lerp(polygonB.vertices[idx0B], polygonB.vertices[idx1B], t[k]);
                    auto pivotA = to_object_space(pivotA_world, posA, ornA);
                    auto pivotB = to_object_space(pivotB_world, posB, ornB);
                    result.maybe_add_point({pivotA, pivotB, sep_axis, distance, normal_attachment});
                }
            }
        }
    }
}

}
