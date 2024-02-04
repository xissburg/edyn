#include "edyn/collision/collide.hpp"
#include "edyn/collision/collision_result.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/math/transform.hpp"
#include "edyn/math/vector2_3_util.hpp"
#include "edyn/math/math.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/util/shape_util.hpp"

namespace edyn {

void collide(const polyhedron_shape &shA, const box_shape &shB,
             const collision_context &ctx, collision_result &result) {
    // Convex polyhedron against box SAT. All calculations done in the
    // polyhedron's space.
    const auto posB = to_object_space(ctx.posB, ctx.posA, ctx.ornA);
    const auto ornB = conjugate(ctx.ornA) * ctx.ornB;
    const auto threshold = ctx.threshold;
    const auto &meshA = *shA.mesh;

    const auto box_axes = std::array<vector3, 3>{
        quaternion_x(ornB),
        quaternion_y(ornB),
        quaternion_z(ornB)
    };

    auto distance = -EDYN_SCALAR_MAX;
    auto projection_poly = scalar(0);
    auto sep_axis = vector3_zero;

    // Face normals of polyhedron.
    for (auto face_idx : meshA.relevant_faces) {
        auto normalA = -meshA.normals[face_idx]; // Point towards polyhedron.
        auto vertexA = meshA.vertices[meshA.first_vertex_index(face_idx)];

        // Find point on box that's furthest along the opposite direction
        // of the face normal.
        auto projA = dot(vertexA, normalA);
        auto projB = shB.support_projection(posB, ornB, normalA);
        auto dist = projA - projB;

        if (dist > distance) {
            distance = dist;
            projection_poly = projA;
            sep_axis = normalA;
        }
    }

    // Face normals of box.
    for (size_t i = 0; i < 3; ++i) {
        auto dir = box_axes[i];

        if (dot(posB, dir) > 0) {
            dir = -dir; // Point towards polyhedron.
        }

        // Find point on polyhedron that's furthest along the opposite direction
        // of the box face normal.
        auto projA = -polyhedron_support_projection(meshA.vertices, meshA.neighbors_start, meshA.neighbor_indices, -dir);
        auto projB = dot(posB, dir) + shB.half_extents[i];
        auto dist = projA - projB;

        if (dist > distance) {
            distance = dist;
            projection_poly = projA;
            sep_axis = dir;
        }
    }

    // Edge vs edge.
    scalar min_edge_dist = -EDYN_SCALAR_MAX;
    scalar edge_projectionA, edge_projectionB;
    auto edge_dir = vector3_zero;

    for (auto edge_idxA = 0u; edge_idxA < meshA.num_edges(); ++edge_idxA) {
        auto vertex_idxA = meshA.get_edge_vertices(edge_idxA);
        auto face_idxA = meshA.get_edge_faces(edge_idxA);

        vector3 normalsA[] = {meshA.normals[face_idxA[0]], meshA.normals[face_idxA[1]]};
        vector3 verticesA[] = {meshA.vertices[vertex_idxA[0]],
                               meshA.vertices[vertex_idxA[1]]};
        auto edge_dirA = verticesA[0] - verticesA[1];

        for (auto edge_idxB = 0u; edge_idxB < get_box_num_features(box_feature::edge); ++edge_idxB) {
            auto normalsB = shB.get_edge_face_normals(edge_idxB, ornB);
            auto verticesB = shB.get_edge(edge_idxB, posB, ornB);
            auto edge_dirB = verticesB[0] - verticesB[1];

            if (edges_generate_minkowski_face(normalsA[0], normalsA[1],
                                              normalsB[0], normalsB[1],
                                              edge_dirA, edge_dirB))
            {
                auto dir = cross(edge_dirA, edge_dirB);

                if (try_normalize(dir)) {
                    // Make direction point outside of shape A.
                    if (dot(verticesA[0], dir) < 0) {
                        dir *= -1;
                    }

                    auto edge_dist = dot(verticesB[0] - verticesA[0], dir);

                    if (edge_dist > min_edge_dist) {
                        min_edge_dist = edge_dist;
                        // Make it point towards A as per the global standard.
                        dir *= -1;
                        edge_projectionA = dot(verticesA[0], dir);
                        edge_projectionB = dot(verticesB[0], dir);
                        edge_dir = dir;
                    }
                }
            }
        }
    }

    if (edge_dir != vector3_zero) {
        auto edge_distance = edge_projectionA - edge_projectionB;

        if (edge_distance > distance) {
            distance = edge_distance;
            projection_poly = edge_projectionA;
            sep_axis = edge_dir;
        }
    }

    if (distance > threshold) {
        return;
    }

    auto polygon = point_cloud_support_polygon(
        meshA.vertices.begin(), meshA.vertices.end(), vector3_zero,
        sep_axis, projection_poly, true, support_feature_tolerance);

    box_feature featureB;
    size_t feature_indexB;
    shB.support_feature(posB, ornB, sep_axis, featureB, feature_indexB,
                        support_feature_tolerance);

    collision_result::collision_point point;
    // Separating axis is in A's space. Transform to global.
    point.normal = rotate(ctx.ornA, sep_axis);
    point.distance = distance;
    point.featureB = {featureB, feature_indexB};

    switch (featureB) {
    case box_feature::face: {
        auto face_verticesB = shB.get_face(feature_indexB, posB, ornB);
        point.normal_attachment = contact_normal_attachment::normal_on_B;

        // Check if vertices of the polyhedron on the contact plane are inside
        // the box face.
        for (auto idxA : polygon.hull) {
            auto &pointA = polygon.vertices[idxA];

            if (point_in_polygonal_prism(face_verticesB, sep_axis, pointA)) {
                point.distance = dot(pointA - face_verticesB[0], sep_axis);
                auto pivotB_world = pointA - sep_axis * point.distance; // Project onto face.
                point.pivotA = pointA;
                point.pivotB = to_object_space(pivotB_world, posB, ornB);
                result.maybe_add_point(point);
            }
        }

        // If the vertices of the polyhedron at the contact plane form a polygon,
        // check if the vertices of the box face are inside the polygon.
        if (polygon.hull.size() > 2) {
            for (auto &pointB : face_verticesB) {
                if (point_in_polygonal_prism(polygon.vertices, polygon.hull, sep_axis, pointB)) {
                    point.distance = dot(polygon.origin - pointB, sep_axis);
                    point.pivotA = pointB + sep_axis * point.distance; // Project onto polygon.
                    point.pivotB = to_object_space(pointB, posB, ornB);
                    result.maybe_add_point(point);
                }
            }
        }

        if (polygon.hull.size() > 1) {
            // Check if the edges of the box face intersect the edges of the polygon.
            std::array<vector2, 4> plane_vertices_box;
            for (size_t i = 0; i < 4; ++i) {
                auto vertex_tangent = to_object_space(face_verticesB[i], polygon.origin, polygon.basis);
                plane_vertices_box[i] = to_vector2_xz(vertex_tangent);
            }

            // If the feature is a polygon, it will be necessary to wrap around the
            // vertex array. If it is just one edge, then avoid calculating the same
            // segment-segment intersection twice.
            const auto sizeA = polygon.hull.size();
            const auto limitA = sizeA == 2 ? 1 : sizeA;
            scalar s[2], t[2];

            for (size_t i = 0; i < limitA; ++i) {
                auto idx0A = polygon.hull[i];
                auto idx1A = polygon.hull[(i + 1) % sizeA];
                auto &v0A = polygon.plane_vertices[idx0A];
                auto &v1A = polygon.plane_vertices[idx1A];

                for (size_t j = 0; j < 4; ++j) {
                    auto idx0B = j;
                    auto idx1B = (j + 1) % 4;
                    auto &v0B = plane_vertices_box[idx0B];
                    auto &v1B = plane_vertices_box[idx1B];
                    auto num_points = intersect_segments(v0A, v1A, v0B, v1B,
                                                         s[0], t[0], s[1], t[1]);

                    for (size_t k = 0; k < num_points; ++k) {
                        point.pivotA = lerp(polygon.vertices[idx0A], polygon.vertices[idx1A], s[k]);
                        auto pivotB_world = lerp(face_verticesB[idx0B], face_verticesB[idx1B], t[k]);
                        point.pivotB = to_object_space(pivotB_world, posB, ornB);
                        result.maybe_add_point(point);
                    }
                }
            }
        }
        break;
    }
    case box_feature::edge: {
        auto edge_vertices_local = shB.get_edge(feature_indexB);
        auto edge_vertices = shB.get_edge(feature_indexB, posB, ornB);
        point.normal_attachment = polygon.hull.size() > 2 ?
            contact_normal_attachment::normal_on_A:
            contact_normal_attachment::none;

        // Check if the vertices of the box edge are inside the polygon.
        if (polygon.hull.size() > 2) {
            for (auto &pointB : edge_vertices) {
                if (point_in_polygonal_prism(polygon.vertices, polygon.hull, sep_axis, pointB)) {
                    point.pivotA = project_plane(pointB, polygon.origin, sep_axis);
                    point.pivotB = to_object_space(pointB, posB, ornB);
                    result.add_point(point);
                }
            }
        }

        // Check if the box edge intersects the edges of the polygon.
        if (polygon.hull.size() > 1) {
            // If the feature is a polygon, it will be necessary to wrap around the
            // vertex array. If it is just one edge, then avoid calculating the same
            // segment-segment intersection twice.
            const auto sizeA = polygon.hull.size();
            const auto limitA = sizeA == 2 ? 1 : sizeA;

            // Vertices of box edge on the contact plane space.
            auto t0B = to_object_space(edge_vertices[0], polygon.origin, polygon.basis);
            auto t1B = to_object_space(edge_vertices[1], polygon.origin, polygon.basis);
            auto v0B = to_vector2_xz(t0B);
            auto v1B = to_vector2_xz(t1B);

            scalar s[2], t[2];

            for (size_t i = 0; i < limitA; ++i) {
                auto idx0A = polygon.hull[i];
                auto idx1A = polygon.hull[(i + 1) % sizeA];
                auto &v0A = polygon.plane_vertices[idx0A];
                auto &v1A = polygon.plane_vertices[idx1A];

                auto num_points = intersect_segments(v0A, v1A, v0B, v1B,
                                                     s[0], t[0], s[1], t[1]);

                for (size_t k = 0; k < num_points; ++k) {
                    point.pivotA = lerp(polygon.vertices[idx0A], polygon.vertices[idx1A], s[k]);
                    point.pivotB = lerp(edge_vertices_local[0], edge_vertices_local[1], t[k]);
                    result.add_point(point);
                }
            }
        } else {
            // Polygon vertex against box edge.
            EDYN_ASSERT(polygon.hull.size() == 1);
            point.pivotA = polygon.vertices[polygon.hull[0]];
            auto edge_dir = edge_vertices[1] - edge_vertices[0];
            vector3 pivotB_world; scalar t;
            closest_point_line(edge_vertices[0], edge_dir, point.pivotA, t, pivotB_world);
            point.pivotB = lerp(edge_vertices_local[0], edge_vertices_local[1], t);
            result.add_point(point);
        }
        break;
    }
    case box_feature::vertex: {
        point.pivotB = shB.get_vertex(feature_indexB);
        auto pivotB_world = to_world_space(point.pivotB, posB, ornB);
        point.pivotA = pivotB_world + sep_axis * distance;
        point.normal_attachment = polygon.hull.size() > 2 ?
            contact_normal_attachment::normal_on_A:
            contact_normal_attachment::none;
        result.add_point(point);
    }
    }
}

void collide(const box_shape &shA, const polyhedron_shape &shB,
             const collision_context &ctx, collision_result &result) {
    swap_collide(shA, shB, ctx, result);
}

}
