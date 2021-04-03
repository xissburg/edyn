#include "edyn/collision/collide.hpp"
#include "edyn/shapes/triangle_shape.hpp"
#include "edyn/util/shape_util.hpp"
#include "edyn/math/vector2_3_util.hpp"

namespace edyn {

void collide_polyhedron_triangle(const polyhedron_shape &poly, rotated_mesh &rmesh,
                                 const triangle_shape &tri,
                                 scalar threshold, collision_result &result) {

    const auto tri_edges = get_triangle_edges(tri.vertices);
    const auto tri_normal = normalize(cross(tri_edges[0], tri_edges[1]));
    bool is_concave_vertex[3];

    for (int i = 0; i < 3; ++i) {
        // If edge starting or ending in this vertex are concave then thus is the vertex.
        is_concave_vertex[i] = tri.is_concave_edge[i] || tri.is_concave_edge[(i + 2) % 3];
    }

    scalar poly_max_proj = EDYN_SCALAR_MAX;
    scalar tri_max_proj = -EDYN_SCALAR_MAX;
    triangle_feature tri_feature;
    size_t tri_feature_index;
    scalar max_distance = -EDYN_SCALAR_MAX;
    auto best_dir = vector3_zero;

    // Polyhedron face normals.
    for (size_t i = 0; i < poly.mesh->num_faces(); ++i) {
        auto normal = -rmesh.normals[i]; // Point towards polyhedron.
        auto vertex_idx = poly.mesh->indices[poly.mesh->faces[i * 2]];
        auto &poly_vertex = rmesh.vertices[vertex_idx];
        auto poly_proj = dot(poly_vertex, -normal);

        // Find point on triangle that's furthest along the opposite direction
        // of the face normal.
        triangle_feature feature;
        size_t feature_idx;
        scalar tri_proj;
        get_triangle_support_feature(tri.vertices, vector3_zero, normal, 
                                     feature, feature_idx, 
                                     tri_proj, threshold);

        auto tri_sup = tri_proj * normal;
        auto dist = dot(tri_sup - poly_vertex, normal);

        if (dist > max_distance) {
            max_distance = dist;
            poly_max_proj = poly_proj;
            tri_max_proj = tri_proj;
            tri_feature = feature;
            tri_feature_index = feature_idx;
            best_dir = normal;
        }
    }

    // Triangle face normal.
    {
        auto tri_proj = dot(tri.vertices[0], tri_normal);

        // Find point on triangle that's furthest along the opposite direction
        // of the face normal.
        auto poly_sup = point_cloud_support_point(rmesh.vertices, -tri_normal);
        auto poly_proj = dot(poly_sup, tri_normal);
        auto dist = dot(poly_sup - tri.vertices[0], tri_normal);

        if (dist > max_distance) {
            max_distance = dist;
            poly_max_proj = poly_proj;
            tri_max_proj = tri_proj;
            tri_feature = triangle_feature::face;
            best_dir = tri_normal;
        }
    }

    // Edge vs edge.
    for (size_t i = 0; i < poly.mesh->edges.size(); i += 2) {
        auto vertexA0 = rmesh.vertices[poly.mesh->edges[i + 0]];
        auto vertexA1 = rmesh.vertices[poly.mesh->edges[i + 1]];
        auto poly_edge = vertexA1 - vertexA0;

        for (size_t j = 0; j < tri_edges.size(); ++j) {
            auto &vertexB0 = tri.vertices[j];
            auto dir = cross(poly_edge, tri_edges[j]);
            auto dir_len_sqr = length_sqr(dir);

            if (dir_len_sqr > EDYN_EPSILON) {
                dir /= std::sqrt(dir_len_sqr);
            } else {
                // Edges are parallel. Find a direction that's orthogonal to both
                // if they don't lie on the same line. Get point in line containing
                // `edgeA` that's closest to `vertexB0`.
                vector3 closest; scalar t;
                closest_point_line(vertexA0, poly_edge, vertexB0, t, closest);

                dir = closest - vertexB0;
                dir_len_sqr = length_sqr(dir);

                if (dir_len_sqr > EDYN_EPSILON) {
                    dir /= std::sqrt(dir_len_sqr);
                } else {
                    continue;
                }
            }

            // Polyhedron is assumed to be located at the origin.
            if (dot(vector3_zero - vertexB0, dir) < 0) {
                // Make it point towards A.
                dir *= -1;
            }

            triangle_feature feature;
            size_t feature_idx;
            scalar tri_proj;
            get_triangle_support_feature(tri.vertices, vector3_zero, dir, 
                                         feature, feature_idx, 
                                         tri_proj, threshold);
            
            auto tri_sup = tri_proj * dir;
            auto poly_sup = point_cloud_support_point(rmesh.vertices, -dir);
            auto distance = dot(poly_sup - tri_sup, dir);

            if (distance > max_distance) {
                max_distance = distance;
                poly_max_proj = dot(poly_sup, dir);
                tri_max_proj = dot(tri_sup, dir);
                tri_feature = feature;
                tri_feature_index = feature_idx;
                best_dir = dir;
            }
        }
    }

    if (max_distance > threshold) {
        return;
    }

    scalar tolerance = 0.002;
    // Find all vertices that are near the projection boundary.
    std::vector<vector3> verticesA;
    // Vertices on the 2D contact plane.
    std::vector<vector2> plane_verticesA;
    // Points at the contact plane.
    auto contact_originA = best_dir * poly_max_proj;
    auto contact_originB = best_dir * tri_max_proj;
    // Build a basis tangent to the contact plane so calculations can be done
    // in tangent space.
    vector3 contact_tangent0, contact_tangent1;
    plane_space(best_dir, contact_tangent0, contact_tangent1);
    auto contact_basis = matrix3x3_columns(contact_tangent0, best_dir, contact_tangent1);

    for (auto &vertex_world : rmesh.vertices) {
        if (dot(vertex_world, best_dir) < poly_max_proj + tolerance) {
            auto vertex_tangent = to_object_space(vertex_world, contact_originA, contact_basis);
            auto vertex_plane = to_vector2_xz(vertex_tangent);
            plane_verticesA.push_back(vertex_plane);
            verticesA.push_back(vertex_world);
        }
    }

    EDYN_ASSERT(!verticesA.empty() && !plane_verticesA.empty());

    auto hullA = calculate_convex_hull(plane_verticesA, tolerance);

    // First, add contact points for vertices that lie inside the opposing face.
    // If the closest triangle feature is its face, check if the vertices of the
    // convex hull of the closest vertices of the polyhedron lie within the 
    // triangle.
    if (tri_feature == triangle_feature::face) {
        for (auto idxA : hullA) {
            auto &pointA = verticesA[idxA];

            if (point_in_triangle(tri.vertices, best_dir, pointA)) {
                auto pivotA = to_object_space(pointA, posA, ornA);
                auto pivotB_world = project_plane(pointA, contact_originB, best_dir);
                auto pivotB = to_object_space(pivotB_world, posB, ornB);
                result.maybe_add_point({pivotA, pivotB, best_dir, max_distance});
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
                result.maybe_add_point({pivotA, pivotB, best_dir, max_distance});
            }
        }
    }

    // Calculate 2D intersection of edges on the closest features.
    if (hullA.size() > 1 && (tri_feature == triangle_feature::edge || 
                             tri_feature == triangle_feature::face)) {
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
}

}