#include "edyn/collision/collide.hpp"
#include "edyn/collision/collision_result.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/math/scalar.hpp"
#include "edyn/math/vector2_3_util.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/util/shape_util.hpp"

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
        auto projB = scalar{};

        auto supB = point_cloud_support_point(shB.mesh->vertices.begin(), shB.mesh->vertices.end(), 
                                              posB, ornB, -normal_world, &projB);
        projB *= -1;

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

collision_result collide(const polyhedron_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const polyhedron_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold) {

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
    for (size_t i = 0; i < shA.mesh->edges.size(); ++i) {
        auto vertexA0 = to_world_space(shA.mesh->vertices[shA.mesh->edges[i * 2 + 0]], posA, ornA);
        auto vertexA1 = to_world_space(shA.mesh->vertices[shA.mesh->edges[i * 2 + 1]], posA, ornA);
        auto edgeA = vertexA1 - vertexA0;

        for (size_t j = 0; j < shB.mesh->edges.size(); ++j) {
            auto vertexB0 = to_world_space(shB.mesh->vertices[shB.mesh->edges[i * 2 + 0]], posB, ornB);
            auto vertexB1 = to_world_space(shB.mesh->vertices[shB.mesh->edges[i * 2 + 1]], posB, ornB);
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

            auto projA = scalar{};
            auto projB = scalar{};

            auto supA = point_cloud_support_point(shA.mesh->vertices.begin(), 
                                                  shA.mesh->vertices.end(), 
                                                  posA, ornA, -dir, &projA);
            auto supB = point_cloud_support_point(shB.mesh->vertices.begin(), 
                                                  shB.mesh->vertices.end(), 
                                                  posB, ornB, dir, &projB);
            auto distance = dot(supA - supB, dir);

            if (distance > max_distance) {
                max_distance = distance;
                projectionA = projA;
                projectionB = projB;
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
    // Vertices on the contact plane.
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

    // Calculate 2D convex hull of contact polygon.
    auto hullA = calculate_convex_hull(plane_verticesA, tolerance);
    auto hullB = calculate_convex_hull(plane_verticesB, tolerance);

    // Calculate 2D intersection of contact polygons, which is the contact area.
    // First, add contact points for vertices that lie inside the opposing face.
    if (verticesA.size() > 2) {
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

    if (verticesB.size() > 2) {
        for (auto idxB : hullB) {
            auto &pointB = verticesB[idxB];

            if (point_in_polygonal_prism(verticesA, hullA, best_dir, pointB)) {
                auto pivotB = to_object_space(pointB, posB, ornB);
                auto pivotA_world = project_plane(pointB, contact_originA, -best_dir);
                auto pivotA = to_object_space(pivotA_world, posA, ornA);
                result.maybe_add_point({pivotA, pivotB, normalB, max_distance});
            }
        }
    }

    return result;
}

}
