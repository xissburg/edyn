#include "edyn/collision/collide.hpp"
#include "edyn/math/scalar.hpp"
#include "edyn/shapes/triangle_shape.hpp"
#include "edyn/util/shape_util.hpp"
#include "edyn/math/vector2_3_util.hpp"
#include "edyn/math/math.hpp"

namespace edyn {

void collide_polyhedron_triangle(const polyhedron_shape &poly, rotated_mesh &rmesh,
                                 const vector3 &pos_poly, const quaternion &orn_poly, 
                                 const triangle_shape &tri,
                                 scalar threshold, collision_result &result) {
    // The triangle vertices are assumed to be shifted by the polyhedron position
    // so all calculations happen near the origin. That means the polyhedron is
    // assumed to be with its pivot at the origin. That also means collision points
    // on the triangle must be shifted by `pos_poly` when inserting then into the
    // `result`.
    scalar poly_max_proj = EDYN_SCALAR_MAX;
    scalar max_distance = -EDYN_SCALAR_MAX;
    scalar tri_max_proj = -EDYN_SCALAR_MAX;
    triangle_feature tri_feature;
    size_t tri_feature_index;
    auto best_dir = vector3_zero;

    // Polyhedron face normals.
    for (size_t i = 0; i < poly.mesh->num_faces(); ++i) {
        auto normal = -rmesh.normals[i]; // Point towards polyhedron.
        auto vertex_idx = poly.mesh->indices[poly.mesh->faces[i * 2]];
        auto &poly_vertex = rmesh.vertices[vertex_idx];

        // Find point on triangle that's furthest along the opposite direction
        // of the face normal.
        triangle_feature feature;
        size_t feature_idx;
        scalar tri_proj;
        get_triangle_support_feature(tri.vertices, vector3_zero, normal, feature, 
                                     feature_idx, tri_proj, threshold);

        if (tri.ignore_feature(feature, feature_idx, normal)) {
            continue;
        }

        auto dist = dot(poly_vertex - normal * tri_proj, normal);

        if (dist > max_distance) {
            max_distance = dist;
            poly_max_proj = dot(poly_vertex, normal);
            tri_max_proj = tri_proj;
            tri_feature = feature;
            tri_feature_index = feature_idx;
            best_dir = normal;
        }
    }

    // Triangle face normal.
    {
        // Find point on polyhedron that's furthest along the opposite direction
        // of the triangle normal.
        auto poly_sup = point_cloud_support_point(rmesh.vertices, -tri.normal);
        auto dist = dot(poly_sup - tri.vertices[0], tri.normal);

        if (dist > max_distance) {
            max_distance = dist;
            poly_max_proj = dot(poly_sup, tri.normal);
            tri_max_proj = dot(tri.vertices[0], tri.normal);
            tri_feature = triangle_feature::face;
            best_dir = tri.normal;
        }
    }

    // Edge vs edge.
    for (size_t i = 0; i < poly.mesh->edges.size(); i += 2) {
        auto vertexA0 = rmesh.vertices[poly.mesh->edges[i + 0]];
        auto vertexA1 = rmesh.vertices[poly.mesh->edges[i + 1]];
        auto poly_edge = vertexA1 - vertexA0;

        for (size_t j = 0; j < tri.edges.size(); ++j) {
            auto &vertexB0 = tri.vertices[j];
            auto dir = cross(poly_edge, tri.edges[j]);
            auto dir_len_sqr = length_sqr(dir);

            if (dir_len_sqr > EDYN_EPSILON) {
                dir /= std::sqrt(dir_len_sqr);
            } else {
                // Edges are parallel. Find a direction that's orthogonal to both
                // if they don't lie on the same line. They lie on the same plane.
                // Find plane normal.
                auto n = cross(vertexB0 - vertexA0, poly_edge);

                if (!(length_sqr(n) > EDYN_EPSILON)) {
                    // Edges are on the same line.
                    continue;
                }

                // Find direction orthogonal to both.
                dir = cross(n, poly_edge);
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
            get_triangle_support_feature(tri.vertices, vector3_zero, dir, feature, 
                                         feature_idx, tri_proj, threshold);

            if (tri.ignore_feature(feature, feature_idx, dir)) {
                continue;
            }

            auto poly_sup = point_cloud_support_point(rmesh.vertices, -dir);
            auto dist = dot(poly_sup - dir * tri_proj, dir);

            if (dist > max_distance) {
                max_distance = dist;
                poly_max_proj = dot(poly_sup, dir);
                tri_max_proj = tri_proj;
                tri_feature = feature;
                tri_feature_index = feature_idx;
                best_dir = dir;
            }
        }
    }

    if (max_distance > threshold) {
        return;
    }

    // Find all vertices that are near the projection boundary.
    scalar tolerance = 0.002;
    std::vector<vector3> vertices_poly;
    // Vertices on the 2D contact plane.
    std::vector<vector2> plane_vertices_poly;
    // Points at the contact plane.
    auto contact_origin_poly = best_dir * poly_max_proj;
    auto contact_origin_tri = best_dir * tri_max_proj;
    // Build a basis tangent to the contact plane so calculations can be done
    // in tangent space.
    vector3 contact_tangent0, contact_tangent1;
    plane_space(best_dir, contact_tangent0, contact_tangent1);
    auto contact_basis = matrix3x3_columns(contact_tangent0, best_dir, contact_tangent1);

    for (auto &vertex_world : rmesh.vertices) {
        if (dot(vertex_world, best_dir) < poly_max_proj + tolerance) {
            auto vertex_tangent = to_object_space(vertex_world, contact_origin_poly, contact_basis);
            auto vertex_plane = to_vector2_xz(vertex_tangent);
            plane_vertices_poly.push_back(vertex_plane);
            vertices_poly.push_back(vertex_world);
        }
    }

    EDYN_ASSERT(!vertices_poly.empty() && !plane_vertices_poly.empty());

    auto hull_poly = calculate_convex_hull(plane_vertices_poly, tolerance);

    auto hull_tri = std::array<size_t, 3>{};
    size_t hull_tri_size = 0;

    switch (tri_feature) {
    case triangle_feature::face:
        std::iota(hull_tri.begin(), hull_tri.end(), 0);
        hull_tri_size = 3;
        break;
    case triangle_feature::edge:
        hull_tri[0] = tri_feature_index;
        hull_tri[1] = (tri_feature_index + 1) % 3;
        hull_tri_size = 2;
        break;
    case triangle_feature::vertex:
        hull_tri[0] = tri_feature_index;
        hull_tri_size = 1;
    }

    auto plane_vertices_tri = std::array<vector2, 3>{};
    for (size_t i = 0; i < 3; ++i) {
        auto &vertex_world = tri.vertices[i];
        auto vertex_tangent = to_object_space(vertex_world, contact_origin_tri, contact_basis);
        auto vertex_plane = to_vector2_xz(vertex_tangent);
        plane_vertices_tri[i] = vertex_plane;
    }

    // If the closest triangle feature is its face, check if the vertices of the
    // convex hull of the closest vertices of the polyhedron lie within the 
    // triangle.
    if (tri_feature == triangle_feature::face) {
        for (auto idxA : hull_poly) {
            auto &pointA = vertices_poly[idxA];

            if (point_in_triangle(tri.vertices, best_dir, pointA)) {
                auto pivotA = to_object_space(pointA, vector3_zero, orn_poly);
                auto pivotB = project_plane(pointA, contact_origin_tri, best_dir) + pos_poly;
                result.maybe_add_point({pivotA, pivotB, best_dir, max_distance});
            }
        }
    }

    // If the boundary points of the polyhedron from a polygon (i.e. more than
    // 2 points) add contact points for the vertices of closest triangle feature
    // that lie inside of it.
    if (hull_poly.size() > 2) {
        for (size_t i = 0; i < hull_tri_size; ++i) {
            auto idxB = hull_tri[i];
            if (tri.ignore_vertex(idxB, best_dir)) continue;

            auto &pointB = tri.vertices[idxB];

            if (point_in_polygonal_prism(vertices_poly, hull_poly, best_dir, pointB)) {
                auto pivotB = pointB + pos_poly;
                auto pivotA_world = project_plane(pointB, contact_origin_poly, best_dir);
                auto pivotA = to_object_space(pivotA_world, vector3_zero, orn_poly);
                result.maybe_add_point({pivotA, pivotB, best_dir, max_distance});
            }
        }
    }

    // Calculate 2D intersection of edges on the closest features.
    if (hull_poly.size() > 1 && hull_tri_size > 1) {
        // If the feature is a polygon, it is will be necessary to wrap around the 
        // vertex array. If it is just one edge, then avoid calculating the same
        // segment-segment intersection twice.
        const auto size_poly = hull_poly.size();
        const auto limit_poly = size_poly == 2 ? 1 : size_poly;
        const auto limit_tri = hull_tri_size == 2 ? 1 : hull_tri_size;
        scalar s[2], t[2];

        for (size_t i = 0; i < limit_poly; ++i) {
            auto idx0A = hull_poly[i];
            auto idx1A = hull_poly[(i + 1) % size_poly];
            auto &v0A = plane_vertices_poly[idx0A];
            auto &v1A = plane_vertices_poly[idx1A];

            for (size_t j = 0; j < limit_tri; ++j) {
                auto idx0B = hull_tri[j];
                if (tri.ignore_edge(idx0B, best_dir)) continue;

                auto idx1B = hull_tri[(j + 1) % hull_tri_size];
                auto &v0B = plane_vertices_tri[idx0B];
                auto &v1B = plane_vertices_tri[idx1B];
                auto num_points = intersect_segments(v0A, v1A, v0B, v1B, 
                                                     s[0], t[0], s[1], t[1]);

                for (size_t k = 0; k < num_points; ++k) {
                    auto pivotA_world = lerp(vertices_poly[idx0A], vertices_poly[idx1A], s[k]);
                    auto pivotB_world = lerp(tri.vertices[idx0B], tri.vertices[idx1B], t[k]);
                    auto pivotA = to_object_space(pivotA_world, vector3_zero, orn_poly);
                    auto pivotB = pivotB_world + pos_poly;
                    result.maybe_add_point({pivotA, pivotB, best_dir, max_distance});
                }
            }
        }
    }
}

}