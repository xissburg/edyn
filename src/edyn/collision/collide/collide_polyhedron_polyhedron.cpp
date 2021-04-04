#include "edyn/collision/collide.hpp"
#include "edyn/collision/collision_result.hpp"
#include "edyn/math/math.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/math/vector2_3_util.hpp"
#include "edyn/util/shape_util.hpp"
#include "edyn/comp/rotated_mesh.hpp"

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

    for (size_t i = 0; i < shA.mesh->num_faces(); ++i) {
        auto &normal_world = rotatedA.normals[i];

        auto vertex_idx = shA.mesh->first_vertex_index(i);
        auto &vertexA = rotatedA.vertices[vertex_idx];
        auto vertex_world = vertexA + posA;

        auto projA = dot(vertex_world, normal_world);

        // Find point on B that's furthest along the opposite direction
        // of the face normal.
        auto supB = point_cloud_support_point(rotatedB.vertices, -normal_world) + posB;
        auto projB = dot(supB, normal_world);

        auto dist = dot(supB - vertex_world, normal_world);

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

collision_result collide(const polyhedron_shape &shA, const polyhedron_shape &shB, 
                         const collision_context &ctx) {
    // Calculate collision with shape A in the origin for better floating point
    // precision. Position of shape B is modified accordingly.
    const auto posA = vector3_zero;
    const auto &ornA = ctx.ornA;
    const auto posB = ctx.posB - ctx.posA;
    const auto &ornB = ctx.ornB;
    const auto threshold = ctx.threshold;

    // The pre-rotated vertices and normals are used to avoid rotating vertices
    // every time.
    auto &rmeshA = *(*ctx.rmeshA);
    auto &rmeshB = *(*ctx.rmeshB);

    scalar max_distance = -EDYN_SCALAR_MAX;
    scalar projectionA = EDYN_SCALAR_MAX;
    scalar projectionB = -EDYN_SCALAR_MAX;
    auto sep_axis = vector3_zero;

    // Find best support direction among all face normals of A.
    max_support_direction(shA, rmeshA, posA, shB, rmeshB, posB, 
                          sep_axis, max_distance, projectionA, projectionB);

    sep_axis *= -1; // Make it point towards A.
    projectionA *= -1;
    projectionB *= -1;

    // Find best support direction among all face normals of B.
    {
        auto distance = scalar{};
        auto projA = scalar{};
        auto projB = scalar{};
        auto dir = vector3_zero;
        max_support_direction(shB, rmeshB, posB, shA, rmeshA, posA, 
                              dir, distance, projB, projA);

        if (distance > max_distance) {
            max_distance = distance;
            projectionA = projA;
            projectionB = projB;
            sep_axis = dir;
        }
    }

    // Edge vs edge.
    for (size_t i = 0; i < shA.mesh->edges.size(); i += 2) {
        auto vertexA0 = rmeshA.vertices[shA.mesh->edges[i + 0]];
        auto vertexA1 = rmeshA.vertices[shA.mesh->edges[i + 1]];
        auto edgeA = vertexA1 - vertexA0;

        for (size_t j = 0; j < shB.mesh->edges.size(); j += 2) {
            auto vertexB0 = rmeshB.vertices[shB.mesh->edges[j + 0]] + posB;
            auto vertexB1 = rmeshB.vertices[shB.mesh->edges[j + 1]] + posB;
            auto edgeB = vertexB1 - vertexB0;
            auto dir = cross(edgeA, edgeB);
            auto dir_len_sqr = length_sqr(dir);

            if (dir_len_sqr > EDYN_EPSILON) {
                dir /= std::sqrt(dir_len_sqr);
            } else {
                // Edges are parallel. Find a direction that's orthogonal to both
                // if they don't lie on the same line. Get point in line containing
                // `edgeA` that's closest to `vertexB0`.
                vector3 closest; scalar t;
                closest_point_line(vertexA0, edgeA, vertexB0, t, closest);

                dir = closest - vertexB0;
                dir_len_sqr = length_sqr(dir);

                if (dir_len_sqr > EDYN_EPSILON) {
                    dir /= std::sqrt(dir_len_sqr);
                } else {
                    continue;
                }
            }

            if (dot(posA - posB, dir) < 0) {
                // Make it point towards A.
                dir *= -1;
            }

            auto supA = point_cloud_support_point(rmeshA.vertices, -dir);
            auto supB = point_cloud_support_point(rmeshB.vertices, dir) + posB;
            auto distance = dot(supA - supB, dir);

            if (distance > max_distance) {
                max_distance = distance;
                projectionA = dot(supA, dir);
                projectionB = dot(supB, dir);
                sep_axis = dir;
            }
        }
    }

    if (max_distance > threshold) {
        return {};
    }

    auto result = collision_result{};
    auto normalB = rotate(conjugate(ornB), sep_axis);
    scalar tolerance = 0.002;

    // Find all vertices that are near the projection boundary.
    std::vector<vector3> verticesA, verticesB;
    // Vertices on the 2D contact plane.
    std::vector<vector2> plane_verticesA, plane_verticesB;
    // Points at the contact planes of A and B.
    auto contact_originA = sep_axis * projectionA;
    auto contact_originB = sep_axis * projectionB;
    // Build a basis tangent to the contact plane so calculations can be done
    // in tangent space.
    vector3 contact_tangent0, contact_tangent1;
    plane_space(sep_axis, contact_tangent0, contact_tangent1);
    auto contact_basis = matrix3x3_columns(contact_tangent0, sep_axis, contact_tangent1);

    for (auto &vertex_world : rmeshA.vertices) {
        if (dot(vertex_world, sep_axis) < projectionA + tolerance) {
            auto vertex_tangent = to_object_space(vertex_world, contact_originA, contact_basis);
            auto vertex_plane = to_vector2_xz(vertex_tangent);
            plane_verticesA.push_back(vertex_plane);
            verticesA.push_back(vertex_world);
        }
    }

    for (auto &vertex : rmeshB.vertices) {
        auto vertex_world = vertex + posB;

        if (dot(vertex_world, sep_axis) > projectionB - tolerance) {
            auto vertex_tangent = to_object_space(vertex_world, contact_originB, contact_basis);
            auto vertex_plane = to_vector2_xz(vertex_tangent);
            plane_verticesB.push_back(vertex_plane);
            verticesB.push_back(vertex_world);
        }
    }

    EDYN_ASSERT(!verticesA.empty() && !plane_verticesA.empty());
    EDYN_ASSERT(!verticesB.empty() && !plane_verticesB.empty());

    auto hullA = calculate_convex_hull(plane_verticesA, tolerance);
    auto hullB = calculate_convex_hull(plane_verticesB, tolerance);

    // First, add contact points for vertices that lie inside the opposing face.
    // If the feature on B is a face, i.e. `verticesB` has 3 or more elements,
    // check if the points in `verticesA` lie inside the prism spanned by `verticesB`
    // and `sep_axis`
    if (hullB.size() > 2) {
        for (auto idxA : hullA) {
            auto &pointA = verticesA[idxA];

            if (point_in_polygonal_prism(verticesB, hullB, sep_axis, pointA)) {
                auto pivotA = to_object_space(pointA, posA, ornA);
                auto pivotB_world = project_plane(pointA, contact_originB, sep_axis);
                auto pivotB = to_object_space(pivotB_world, posB, ornB);
                result.maybe_add_point({pivotA, pivotB, normalB, max_distance});
            }
        }
    }

    if (hullA.size() > 2) {
        for (auto idxB : hullB) {
            auto &pointB = verticesB[idxB];

            if (point_in_polygonal_prism(verticesA, hullA, sep_axis, pointB)) {
                auto pivotB = to_object_space(pointB, posB, ornB);
                auto pivotA_world = project_plane(pointB, contact_originA, sep_axis);
                auto pivotA = to_object_space(pivotA_world, posA, ornA);
                result.maybe_add_point({pivotA, pivotB, normalB, max_distance});
            }
        }
    }

    // Calculate 2D intersection of edges on the closest features.
    if (hullA.size() > 1 && hullB.size() > 1) {
        // If the feature is a polygon, it is will be necessary to wrap around the 
        // vertex array. If it is just one edge, then avoid calculating the same
        // segment-segment intersection twice.
        const auto sizeA = hullA.size();
        const auto sizeB = hullB.size();
        const auto limitA = sizeA == 2 ? 1 : sizeA;
        const auto limitB = sizeB == 2 ? 1 : sizeB;
        scalar s[2], t[2];

        for (size_t i = 0; i < limitA; ++i) {
            auto idx0A = hullA[i];
            auto idx1A = hullA[(i + 1) % sizeA];
            auto &v0A = plane_verticesA[idx0A];
            auto &v1A = plane_verticesA[idx1A];

            for (size_t j = 0; j < limitB; ++j) {
                auto idx0B = hullB[j];
                auto idx1B = hullB[(j + 1) % sizeB];
                auto &v0B = plane_verticesB[idx0B];
                auto &v1B = plane_verticesB[idx1B];
                auto num_points = intersect_segments(v0A, v1A, v0B, v1B, 
                                                     s[0], t[0], s[1], t[1]);

                for (size_t k = 0; k < num_points; ++k) {
                    auto pivotA_world = lerp(verticesA[idx0A], verticesA[idx1A], s[k]);
                    auto pivotB_world = lerp(verticesB[idx0B], verticesB[idx1B], t[k]);
                    auto pivotA = to_object_space(pivotA_world, posA, ornA);
                    auto pivotB = to_object_space(pivotB_world, posB, ornB);
                    result.maybe_add_point({pivotA, pivotB, normalB, max_distance});
                }
            }
        }
    }

    return result;
}

}
