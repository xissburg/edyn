#include "edyn/collision/collide.hpp"
#include "edyn/collision/collision_result.hpp"
#include "edyn/math/math.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/math/vector2_3_util.hpp"
#include "edyn/util/shape_util.hpp"
#include <numeric>

namespace edyn {

void max_support_direction(const polyhedron_shape &shA, const vector3 &posA, const quaternion &ornA,
                           const polyhedron_shape &shB, const vector3 &posB, const quaternion &ornB,
                           vector3 &dir, scalar &distance, scalar &projectionA, scalar &projectionB) {
    scalar max_proj_A = EDYN_SCALAR_MAX;
    scalar max_proj_B = -EDYN_SCALAR_MAX;
    scalar max_distance = -EDYN_SCALAR_MAX;
    auto best_dir = vector3_zero;

    for (size_t i = 0; i < shA.mesh->num_triangles(); ++i) {
        auto normalA = shA.mesh->normals[i];
        auto normal_world = rotate(ornA, normalA);

        auto vertex_idx = shA.mesh->indices[i * 3];
        auto &vertexA = shA.mesh->vertices[vertex_idx];
        auto vertex_world = to_world_space(vertexA, posA, ornA);

        auto projA = dot(vertex_world, normal_world);

        // Find point on B that's furthest along the opposite direction
        // of the triangle normal.
        auto supB = point_cloud_support_point(shB.mesh->vertices, 
                                              posB, ornB, -normal_world);
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

void sort_triangle_ccw(vector2 &v0, vector2 &v1, vector2 &v2) {
    auto e = v1 - v0;
    auto t = orthogonal(e);

    if (dot(v2 - v0, t) < 0) {
        std::swap(v0, v2);
    }
}

collision_result collide(const polyhedron_shape &shA, const polyhedron_shape &shB, 
                         const collision_context &ctx) {
    const auto &posA = ctx.posA;
    const auto &ornA = ctx.ornA;
    const auto &posB = ctx.posB;
    const auto &ornB = ctx.ornB;
    const auto threshold = ctx.threshold;

    scalar max_distance = -EDYN_SCALAR_MAX;
    scalar projectionA = EDYN_SCALAR_MAX;
    scalar projectionB = -EDYN_SCALAR_MAX;
    auto best_dir = vector3_zero;

    // Find best support direction among all triangle normals of A.
    max_support_direction(shA, posA, ornA, shB, posB, ornB, best_dir, max_distance, projectionA, projectionB);

    best_dir *= -1; // Make it point towards A.
    projectionA *= -1;
    projectionB *= -1;

    // Find best support direction among all triangle normals of B.
    {
        auto distance = scalar{};
        auto projA = scalar{};
        auto projB = scalar{};
        auto dir = vector3_zero;
        max_support_direction(shB, posB, ornB, shA, posA, ornA, dir, distance, projB, projA);

        if (distance > max_distance) {
            max_distance = distance;
            projectionA = projA;
            projectionB = projB;
            best_dir = dir;
        }
    }

    // Edge vs edge.
    for (size_t i = 0; i < shA.mesh->edges.size(); i += 2) {
        auto vertexA0 = to_world_space(shA.mesh->vertices[shA.mesh->edges[i + 0]], posA, ornA);
        auto vertexA1 = to_world_space(shA.mesh->vertices[shA.mesh->edges[i + 1]], posA, ornA);
        auto edgeA = vertexA1 - vertexA0;

        for (size_t j = 0; j < shB.mesh->edges.size(); j += 2) {
            auto vertexB0 = to_world_space(shB.mesh->vertices[shB.mesh->edges[j + 0]], posB, ornB);
            auto vertexB1 = to_world_space(shB.mesh->vertices[shB.mesh->edges[j + 1]], posB, ornB);
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

            auto supA = point_cloud_support_point(shA.mesh->vertices, 
                                                  posA, ornA, -dir);
            auto supB = point_cloud_support_point(shB.mesh->vertices,
                                                  posB, ornB, dir);
            auto distance = dot(supA - supB, dir);

            if (distance > max_distance) {
                max_distance = distance;
                projectionA = dot(supA, dir);
                projectionB = dot(supB, dir);
                best_dir = dir;
            }
        }
    }

    if (max_distance > threshold) {
        return {};
    }

    auto result = collision_result{};
    auto normalB = rotate(conjugate(ornB), best_dir);
    scalar tolerance = 0.002;

    // Find all vertices that are near the projection boundary.
    std::vector<vector3> verticesA, verticesB;
    // Vertices on the 2D contact plane.
    std::vector<vector2> plane_verticesA, plane_verticesB;
    auto contact_originA = best_dir * projectionA;
    auto contact_originB = best_dir * projectionB;
    vector3 contact_tangent0, contact_tangent1;
    plane_space(best_dir, contact_tangent0, contact_tangent1);
    auto contact_basis = matrix3x3_columns(contact_tangent0, best_dir, contact_tangent1);

    for (auto &vertex : shA.mesh->vertices) {
        auto vertex_world = to_world_space(vertex, posA, ornA);

        if (dot(vertex_world, best_dir) < projectionA + tolerance) {
            auto vertex_tangent = to_object_space(vertex_world, contact_originA, contact_basis);
            auto vertex_plane = to_vector2_xz(vertex_tangent);
            plane_verticesA.push_back(vertex_plane);
            verticesA.push_back(vertex_world);
        }
    }

    for (auto &vertex : shB.mesh->vertices) {
        auto vertex_world = to_world_space(vertex, posB, ornB);

        if (dot(vertex_world, best_dir) > projectionB - tolerance) {
            auto vertex_tangent = to_object_space(vertex_world, contact_originB, contact_basis);
            auto vertex_plane = to_vector2_xz(vertex_tangent);
            plane_verticesB.push_back(vertex_plane);
            verticesB.push_back(vertex_world);
        }
    }

    EDYN_ASSERT(!verticesA.empty());
    EDYN_ASSERT(!verticesB.empty());

    std::vector<size_t> hullA, hullB;

    if (plane_verticesA.size() > 3) {
        // Calculate 2D convex hull of contact polygon.
        hullA = calculate_convex_hull(plane_verticesA, tolerance);
    } else {
        if (plane_verticesA.size() == 3) {
            // It is a triangle, just have to make sure vertices are
            // oriented counter-clockwise.
            sort_triangle_ccw(plane_verticesA[0], plane_verticesA[1], plane_verticesA[2]);
        }

        hullA.resize(plane_verticesA.size());
        std::iota(hullA.begin(), hullA.end(), 0);
    }

    if (plane_verticesB.size() > 3) {
        hullB = calculate_convex_hull(plane_verticesB, tolerance);
    } else {
        if (plane_verticesB.size() == 3) {
            sort_triangle_ccw(plane_verticesB[0], plane_verticesB[1], plane_verticesB[2]);
        }

        hullB.resize(plane_verticesB.size());
        std::iota(hullB.begin(), hullB.end(), 0);
    }

    // First, add contact points for vertices that lie inside the opposing face.
    // If the feature on B is a face, i.e. `verticesB` has 3 or more elements,
    // check if the points in `verticesA` lie inside the prism spanned by `verticesB`
    // and `best_dir`
    if (hullB.size() > 2) {
        for (auto idxA : hullA) {
            auto &pointA = verticesA[idxA];

            if (point_in_polygonal_prism(verticesB, hullB, best_dir, pointA)) {
                auto pivotA = to_object_space(pointA, posA, ornA);
                auto pivotB_world = project_plane(pointA, contact_originB, best_dir);
                auto pivotB = to_object_space(pivotB_world, posB, ornB);
                result.maybe_add_point({pivotA, pivotB, normalB, max_distance});
            }
        }
    }

    if (hullA.size() > 2) {
        for (auto idxB : hullB) {
            auto &pointB = verticesB[idxB];

            if (point_in_polygonal_prism(verticesA, hullA, best_dir, pointB)) {
                auto pivotB = to_object_space(pointB, posB, ornB);
                auto pivotA_world = project_plane(pointB, contact_originA, best_dir);
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
