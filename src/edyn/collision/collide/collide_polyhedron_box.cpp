#include "edyn/collision/collide.hpp"
#include "edyn/collision/collision_result.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/util/shape_util.hpp"
#include "edyn/math/vector2_3_util.hpp"
#include "edyn/math/math.hpp"

namespace edyn {

collision_result collide(const polyhedron_shape &shA, const box_shape &shB,
                         const collision_context &ctx) {
    // Convex polyhedron against box SAT.
    // Calculate collision with polyhedron in the origin for better floating point
    // precision. Position of box is modified accordingly.
    const auto posA = vector3_zero;
    const auto &ornA = ctx.ornA;
    const auto posB = ctx.posB - ctx.posA;
    const auto &ornB = ctx.ornB;
    const auto threshold = ctx.threshold;

    // The pre-rotated vertices and normals are used to avoid rotating vertices
    // every time.
    auto &rmeshA = ctx.rmeshA->get();

    const auto box_axes = std::array<vector3, 3>{
        quaternion_x(ornB),
        quaternion_y(ornB),
        quaternion_z(ornB)
    };

    scalar max_distance = -EDYN_SCALAR_MAX;
    scalar projection_poly = EDYN_SCALAR_MAX;
    scalar projection_box = -EDYN_SCALAR_MAX;
    auto sep_axis = vector3_zero;

    // Face normals of polyhedron.
    for (size_t i = 0; i < shA.mesh->num_faces(); ++i) {
        auto normal_world = -rmeshA.normals[i]; // Point towards polyhedron.

        auto vertex_idx = shA.mesh->first_vertex_index(i);
        auto &vertexA = rmeshA.vertices[vertex_idx];
        auto vertex_world = vertexA + posA;

        // Find point on box that's furthest along the opposite direction
        // of the face normal.
        auto supB = shB.support_point(posB, ornB, normal_world);
        auto dist = dot(vertex_world - supB, normal_world);

        if (dist > max_distance) {
            max_distance = dist;
            projection_poly = dot(vertex_world, normal_world);
            projection_box = dot(supB, normal_world);
            sep_axis = normal_world;
        }
    }

    // Face normals of box.
    for (size_t i = 0; i < 3; ++i) {
        auto dir = box_axes[i];
        if (dot(posA - posB, dir) < 0) {
            dir = -dir; // Point towards polyhedron.
        }

        // Find point on polyhedron that's furthest along the opposite direction
        // of the box face normal.
        auto supA = point_cloud_support_point(rmeshA.vertices, -dir);
        auto supB = posB + dir * shB.half_extents[i];
        auto dist = dot(supA - supB, dir);

        if (dist > max_distance) {
            max_distance = dist;
            projection_poly = dot(supA, dir);
            projection_box = dot(supB, dir);
            sep_axis = dir;
        }
    }

    // Edge vs edge.
    for (size_t i = 0; i < shA.mesh->edges.size(); i += 2) {
        auto vertexA0 = rmeshA.vertices[shA.mesh->edges[i + 0]];
        auto vertexA1 = rmeshA.vertices[shA.mesh->edges[i + 1]];
        auto poly_edge = vertexA1 - vertexA0;

        for (auto &box_edge : box_axes) {
            auto dir = cross(poly_edge, box_edge);
            auto dir_len_sqr = length_sqr(dir);

            if (dir_len_sqr > EDYN_EPSILON) {
                dir /= std::sqrt(dir_len_sqr);
            } else {
                // Edges are parallel. Find a direction that's orthogonal to both
                // if they don't lie on the same line. They lie on the same plane.
                // Calculate plane normal.
                auto n = cross(posB - vertexA0, poly_edge);

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

            if (dot(posA - posB, dir) < 0) {
                // Make it point towards A.
                dir *= -1;
            }

            auto supA = point_cloud_support_point(rmeshA.vertices, -dir);
            auto supB = shB.support_point(posB, ornB, dir);
            auto distance = dot(supA - supB, dir);

            if (distance > max_distance) {
                max_distance = distance;
                projection_poly = dot(supA, dir);
                projection_box = dot(supB, dir);
                sep_axis = dir;
            }
        }
    }

    if (max_distance > threshold) {
        return {};
    }

    // Find all vertices in the polyhedron that are near the projection boundary.
    scalar tolerance = 0.002;
    std::vector<vector3> vertices_poly;
    // Vertices on the 2D contact plane.
    std::vector<vector2> plane_vertices_poly;
    // Points at the contact plane.
    auto contact_origin_poly = sep_axis * projection_poly;
    auto contact_origin_box = sep_axis * projection_box;
    // Build a basis tangent to the contact plane so calculations can be done
    // in tangent space.
    vector3 contact_tangent0, contact_tangent1;
    plane_space(sep_axis, contact_tangent0, contact_tangent1);
    auto contact_basis = matrix3x3_columns(contact_tangent0, sep_axis, contact_tangent1);

    for (auto &vertex_world : rmeshA.vertices) {
        if (dot(vertex_world, sep_axis) < projection_poly + tolerance) {
            auto vertex_tangent = to_object_space(vertex_world, contact_origin_poly, contact_basis);
            auto vertex_plane = to_vector2_xz(vertex_tangent);
            plane_vertices_poly.push_back(vertex_plane);
            vertices_poly.push_back(vertex_world);
        }
    }

    EDYN_ASSERT(!vertices_poly.empty() && !plane_vertices_poly.empty());

    auto hull_poly = calculate_convex_hull(plane_vertices_poly, tolerance);

    scalar feature_distanceB;
    box_feature featureB;
    size_t feature_indexB;
    shB.support_feature(posB, ornB, contact_origin_box, sep_axis, 
                        featureB, feature_indexB,
                        feature_distanceB, threshold);

    auto result = collision_result{};
    auto normalB = rotate(conjugate(ornB), sep_axis);

    switch (featureB) {
    case box_feature::face: {
        auto face_vertices = shB.get_face(feature_indexB, posB, ornB);

        // Check if vertices of the polyhedron on the contact plane are inside
        // the box face.
        for (auto idxA : hull_poly) {
            auto &pointA = vertices_poly[idxA];

            if (point_in_polygonal_prism(face_vertices, sep_axis, pointA)) {
                auto pivotA = to_object_space(pointA, posA, ornA);
                auto pivotB_world = project_plane(pointA, contact_origin_box, sep_axis);
                auto pivotB = to_object_space(pivotB_world, posB, ornB);
                result.maybe_add_point({pivotA, pivotB, normalB, max_distance});
            }
        }

        // If the vertices of the polyhedron at the contact plane form a polygon,
        // check if the vertices of the box face are inside the polygon.
        if (hull_poly.size() > 2) {
            for (auto &pointB : face_vertices) {
                if (point_in_polygonal_prism(vertices_poly, hull_poly, sep_axis, pointB)) {
                    auto pivotB = to_object_space(pointB, posB, ornB);
                    auto pivotA_world = project_plane(pointB, contact_origin_poly, sep_axis);
                    auto pivotA = to_object_space(pivotA_world, posA, ornA);
                    result.maybe_add_point({pivotA, pivotB, normalB, max_distance});
                }
            }
        }

        if (hull_poly.size() > 1) {
            // Check if the edges of the box face intersects the edges of the polygon.
            std::array<vector2, 4> plane_vertices_box;
            for (size_t i = 0; i < 4; ++i) {
                auto vertex_tangent = to_object_space(face_vertices[i], contact_origin_box, contact_basis);
                plane_vertices_box[i] = to_vector2_xz(vertex_tangent);
            }

            // If the feature is a polygon, it is will be necessary to wrap around the 
            // vertex array. If it is just one edge, then avoid calculating the same
            // segment-segment intersection twice.
            const auto sizeA = hull_poly.size();
            const auto limitA = sizeA == 2 ? 1 : sizeA;
            scalar s[2], t[2];

            for (size_t i = 0; i < limitA; ++i) {
                auto idx0A = hull_poly[i];
                auto idx1A = hull_poly[(i + 1) % sizeA];
                auto &v0A = plane_vertices_poly[idx0A];
                auto &v1A = plane_vertices_poly[idx1A];

                for (size_t j = 0; j < 4; ++j) {
                    auto idx0B = j;
                    auto idx1B = (j + 1) % 4;
                    auto &v0B = plane_vertices_box[idx0B];
                    auto &v1B = plane_vertices_box[idx1B];
                    auto num_points = intersect_segments(v0A, v1A, v0B, v1B, 
                                                        s[0], t[0], s[1], t[1]);

                    for (size_t k = 0; k < num_points; ++k) {
                        auto pivotA_world = lerp(vertices_poly[idx0A], vertices_poly[idx1A], s[k]);
                        auto pivotB_world = lerp(face_vertices[idx0B], face_vertices[idx1B], t[k]);
                        auto pivotA = to_object_space(pivotA_world, posA, ornA);
                        auto pivotB = to_object_space(pivotB_world, posB, ornB);
                        result.maybe_add_point({pivotA, pivotB, normalB, max_distance});
                    }
                }
            }
        }
        break;
    }
    case box_feature::edge: {
        auto edge_vertices = shB.get_edge(feature_indexB, posB, ornB);

        // Check if the vertices of the box edge are inside the polygon.
        if (hull_poly.size() > 2) {
            for (auto &pointB : edge_vertices) {
                if (point_in_polygonal_prism(vertices_poly, hull_poly, sep_axis, pointB)) {
                    auto pivotB = to_object_space(pointB, posB, ornB);
                    auto pivotA_world = project_plane(pointB, contact_origin_poly, sep_axis);
                    auto pivotA = to_object_space(pivotA_world, posA, ornA);
                    result.add_point({pivotA, pivotB, normalB, max_distance});
                }
            }
        }

        // Check if the box edge intersects the edges of the polygon.
        if (hull_poly.size() > 1) {
            // If the feature is a polygon, it is will be necessary to wrap around the 
            // vertex array. If it is just one edge, then avoid calculating the same
            // segment-segment intersection twice.
            const auto sizeA = hull_poly.size();
            const auto limitA = sizeA == 2 ? 1 : sizeA;

            // Vertices of box edge on the contact plane space.
            auto t0B = to_object_space(edge_vertices[0], contact_origin_box, contact_basis);
            auto t1B = to_object_space(edge_vertices[1], contact_origin_box, contact_basis);
            auto v0B = to_vector2_xz(t0B);
            auto v1B = to_vector2_xz(t1B);

            scalar s[2], t[2];

            for (size_t i = 0; i < limitA; ++i) {
                auto idx0A = hull_poly[i];
                auto idx1A = hull_poly[(i + 1) % sizeA];
                auto &v0A = plane_vertices_poly[idx0A];
                auto &v1A = plane_vertices_poly[idx1A];

                auto num_points = intersect_segments(v0A, v1A, v0B, v1B, 
                                                    s[0], t[0], s[1], t[1]);

                for (size_t k = 0; k < num_points; ++k) {
                    auto pivotA_world = lerp(vertices_poly[idx0A], vertices_poly[idx1A], s[k]);
                    auto pivotB_world = lerp(edge_vertices[0], edge_vertices[1], t[k]);
                    auto pivotA = to_object_space(pivotA_world, posA, ornA);
                    auto pivotB = to_object_space(pivotB_world, posB, ornB);
                    result.add_point({pivotA, pivotB, normalB, max_distance});
                }
            }
        } else {
            // Polygon vertex against box edge.
            EDYN_ASSERT(hull_poly.size() == 1);
            auto &pivotA = vertices_poly[hull_poly[0]];
            auto edge_dir = edge_vertices[1] - edge_vertices[0];
            vector3 pivotB_world; scalar t;
            closest_point_line(edge_vertices[0], edge_dir, pivotA, t, pivotB_world);
            auto pivotB = to_object_space(pivotB_world, posB, ornB);
            result.add_point({pivotA, pivotB, normalB, max_distance});
        }
        break;
    }
    case box_feature::vertex: {
        auto pivotB = shB.get_vertex(feature_indexB);
        auto pivotB_world = to_world_space(pivotB, posB, ornB);
        auto pivotA_world = pivotB_world + sep_axis * max_distance;
        auto pivotA = to_object_space(pivotA_world, posA, ornA);
        result.add_point({pivotA, pivotB, normalB, max_distance});
    }
    }

    return result;
}

collision_result collide(const box_shape &shA, const polyhedron_shape &shB, 
                         const collision_context &ctx) {
    return swap_collide(shA, shB, ctx);
}

}
