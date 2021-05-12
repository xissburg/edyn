#include "edyn/collision/collide.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/math/math.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/math/vector2_3_util.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/shapes/box_shape.hpp"
#include "edyn/shapes/triangle_shape.hpp"
#include <sys/_types/_size_t.h>

namespace edyn {

void collide(const box_shape &box, const triangle_mesh &mesh, size_t tri_idx,
             const collision_context &ctx, collision_result &result) {
    auto tri = mesh.get_triangle(tri_idx);

    const auto &posA = ctx.posA;
    const auto &ornA = ctx.ornA;
    const auto threshold = ctx.threshold;

    triangle_feature featureB;
    size_t feature_indexB;
    scalar distance = -EDYN_SCALAR_MAX;
    vector3 sep_axis;

    // TODO: Could avoid doing this for each triangle if this function could
    // have the ability to process multiple triangles.
    const vector3 box_axes[] = {
        quaternion_x(ctx.ornA),
        quaternion_y(ctx.ornA),
        quaternion_z(ctx.ornA)
    };

    // Box faces.
    for (size_t i = 0; i < 6; ++i) {
        auto j = i % 3;
        auto dir = box_axes[j] * to_sign(i < 3);

        triangle_feature tri_feature;
        size_t tri_feature_idx;
        scalar proj;
        get_triangle_support_feature(tri.vertices, posA, dir,
                                     tri_feature, tri_feature_idx, proj,
                                     support_feature_tolerance);

        if (tri.ignore_feature(tri_feature, tri_feature_idx, dir)) {
            continue;
        }

        // `proj` is the projection of the support point on the triangle onto
        // `dir` considering it starts at `posA`. The projection of the box
        // onto this axis is the half extent in this direction.
        auto dist = -(box.half_extents[j] + proj);

        if (dist > distance) {
            distance = dist;
            sep_axis = dir;
            featureB = tri_feature;
            feature_indexB = tri_feature_idx;
        }
    }

    // Triangle face normal.
    {
        auto proj = box.support_projection(posA, ornA, -tri.normal);
        auto dist = -(proj + dot(tri.vertices[0], tri.normal));

        if (dist > distance) {
            distance = dist;
            sep_axis = tri.normal;
            featureB = triangle_feature::face;
        }
    }

    // Edges.
    for (size_t i = 0; i < 3; ++i) {
        auto &axisA = box_axes[i];

        for (size_t j = 0; j < 3; ++j) {
            auto &axisB = tri.edges[j];
            auto dir = cross(axisA, axisB);

            if (!try_normalize(dir)) {
                continue;
            }

            if (dot(posA - tri.vertices[j], dir) < 0) {
                dir *= -1; // Make it point towards box.
            }

            triangle_feature tri_feature;
            size_t tri_feature_idx;
            scalar projB;
            get_triangle_support_feature(tri.vertices, vector3_zero, dir,
                                         tri_feature, tri_feature_idx,
                                         projB, support_feature_tolerance);

            if (tri.ignore_feature(tri_feature, tri_feature_idx, dir)) {
                continue;
            }

            auto projA = -box.support_projection(posA, ornA, -dir);
            auto dist = projA - projB;

            if (dist > distance) {
                distance = dist;
                sep_axis = dir;
                featureB = tri_feature;
                feature_indexB = tri_feature_idx;
            }
        }
    }

    // No collision.
    if (distance > threshold) {
        return;
    }

    box_feature featureA;
    size_t feature_indexA;
    scalar projectionA;
    box.support_feature(posA, ornA, vector3_zero, -sep_axis,
                        featureA, feature_indexA, projectionA,
                        support_feature_tolerance);

    if (featureA == box_feature::face && featureB == triangle_feature::face) {
        auto normalA = box.get_face_normal(feature_indexA, ornA);
        auto verticesA_local = box.get_face(feature_indexA);
        std::array<vector3, 4> verticesA;
        for (int i = 0; i < 4; ++i) {
            verticesA[i] = to_world_space(verticesA_local[i], posA, ornA);
        }

        // Check for triangle vertices inside box face.
        size_t num_tri_vert_in_box_face = 0;

        for (int i = 0; i < 3; ++i) {
            // Ignore vertices that are on a concave edge.
            if (tri.is_concave_vertex[i]) {
                continue;
            }

            if (point_in_polygonal_prism(verticesA, normalA, tri.vertices[i])) {
                // Triangle vertex is inside box face.
                auto pivot_on_face = project_plane(tri.vertices[i], verticesA[0], normalA);
                auto pivotA = to_object_space(pivot_on_face, posA, ornA);
                auto pivotB = tri.vertices[i];
                auto local_distance = dot(pivot_on_face - pivotB, sep_axis);
                result.maybe_add_point({pivotA, pivotB, sep_axis, local_distance});
                ++num_tri_vert_in_box_face;
            }
        }

        // If all triangle vertices are contained in the box face, there's
        // nothing else to be done.
        if (num_tri_vert_in_box_face == 3) {
            return;
        }

        // Look for box face vertices inside triangle face.
        size_t num_box_vert_in_tri_face = 0;

        for (int i = 0; i < 4; ++i) {
            if (point_in_triangle(tri.vertices, tri.normal, verticesA[i])) {
                auto pivotA = verticesA_local[i];
                auto pivotB = project_plane(verticesA[i], tri.vertices[0], tri.normal);
                auto local_distance = dot(verticesA[i] - tri.vertices[0], sep_axis);
                result.maybe_add_point({pivotA, pivotB, sep_axis, local_distance});
                ++num_box_vert_in_tri_face;
            }
        }

        // If all box face vertices are contained in the triangle, there's
        // nothing else to be done.
        if (num_box_vert_in_tri_face == 4) {
            return;
        }

        // Perform edge intersection tests.
        for (int i = 0; i < 3; ++i) {
            // Ignore concave edges.
            if (tri.is_concave_edge[i]) {
                continue;
            }

            auto &b0 = tri.vertices[i];
            auto &b1 = tri.vertices[(i + 1) % 3];

            // Convert this into a 2D segment intersection problem in the box' space.
            auto b0_in_A = to_object_space(b0, posA, ornA);
            auto b1_in_A = to_object_space(b1, posA, ornA);

            vector2 half_extents;
            vector2 p0, p1;

            // Pick the planar coordinates based on the face index.
            if (feature_indexA == 0 || feature_indexA == 1) { // X face
                half_extents = to_vector2_zy(box.half_extents);
                p0 = to_vector2_zy(b0_in_A); p1 = to_vector2_zy(b1_in_A);
            } else if (feature_indexA == 2 || feature_indexA == 3) { // Y face
                half_extents = to_vector2_xz(box.half_extents);
                p0 = to_vector2_xz(b0_in_A); p1 = to_vector2_xz(b1_in_A);
            } else { // if (feature_indexA == 4 || feature_indexA == 5) { // Z face
                half_extents = to_vector2_xy(box.half_extents);
                p0 = to_vector2_xy(b0_in_A); p1 = to_vector2_xy(b1_in_A);
            }

            scalar s[2];
            auto num_points = intersect_line_aabb(p0, p1, -half_extents, half_extents, s[0], s[1]);

            for (size_t k = 0; k < num_points; ++k) {
                if (s[k] < 0 || s[k] > 1) continue;

                auto pivotB = lerp(b0, b1, s[k]);
                auto pivotA_world = project_plane(pivotB, verticesA[0], normalA);
                auto pivotA = to_object_space(pivotA_world, posA, ornA);
                auto local_distance = dot(pivotA_world - pivotB, sep_axis);
                result.maybe_add_point({pivotA, pivotB, sep_axis, local_distance});
            }
        }
    } else if (featureA == box_feature::face && featureB == triangle_feature::edge) {
        EDYN_ASSERT(!tri.is_concave_edge[feature_indexB]);

        auto normalA = box.get_face_normal(feature_indexA, ornA);
        auto verticesA_local = box.get_face(feature_indexA);
        std::array<vector3, 4> verticesA;
        for (int i = 0; i < 4; ++i) {
            verticesA[i] = to_world_space(verticesA_local[i], posA, ornA);
        }

        // Check if edge vertices are inside box face.
        vector3 edge_vertices[] = {tri.vertices[feature_indexB],
                                   tri.vertices[(feature_indexB + 1) % 3]};
        size_t num_edge_vert_in_box_face = 0;

        for (int i = 0; i < 2; ++i) {
            if (point_in_polygonal_prism(verticesA, normalA, edge_vertices[i])) {
                // Edge's vertex is inside face.
                auto pivot_on_face = project_plane(edge_vertices[i], verticesA[0], normalA);
                auto pivotA = to_object_space(pivot_on_face, posA, ornA);
                auto pivotB = edge_vertices[i];
                auto local_distance = dot(pivot_on_face - pivotB, sep_axis);
                result.maybe_add_point({pivotA, pivotB, sep_axis, local_distance});
                ++num_edge_vert_in_box_face;
            }
        }

        // If both vertices are inside the face there's nothing else to be done.
        if (num_edge_vert_in_box_face == 2) {
            return;
        }

        auto e0_in_A = to_object_space(edge_vertices[0], posA, ornA);
        auto e1_in_A = to_object_space(edge_vertices[1], posA, ornA);

        vector2 half_extents;
        vector2 p0, p1;

        if (feature_indexA == 0 || feature_indexA == 1) { // X face
            half_extents = to_vector2_zy(box.half_extents);
            p0 = to_vector2_zy(e0_in_A); p1 = to_vector2_zy(e1_in_A);
        } else if (feature_indexA == 2 || feature_indexA == 3) { // Y face
            half_extents = to_vector2_xz(box.half_extents);
            p0 = to_vector2_xz(e0_in_A); p1 = to_vector2_xz(e1_in_A);
        } else { // if (feature_indexA == 4 || feature_indexA == 5) { // Z face
            half_extents = to_vector2_xy(box.half_extents);
            p0 = to_vector2_xy(e0_in_A); p1 = to_vector2_xy(e1_in_A);
        }

        scalar s[2];
        auto num_points = intersect_line_aabb(p0, p1, -half_extents, half_extents, s[0], s[1]);

        for (size_t k = 0; k < num_points; ++k) {
            if (s[k] < 0 || s[k] > 1) continue;

            auto pivotB = lerp(edge_vertices[0], edge_vertices[1], s[k]);
            auto pivotA_world = project_plane(pivotB, verticesA[0], normalA);
            auto pivotA = to_object_space(pivotA_world, posA, ornA);
            auto local_distance = dot(pivotA_world - pivotB, sep_axis);
            result.maybe_add_point({pivotA, pivotB, sep_axis, local_distance});
        }
    } else if (featureA == box_feature::edge && featureB == triangle_feature::face) {
        // Check if edge vertices are inside triangle face.
        auto verticesA_local = box.get_edge(feature_indexA);
        std::array<vector3, 2> verticesA;
        size_t num_edge_vert_in_tri_face = 0;

        for (size_t i = 0; i < 2; ++i) {
            verticesA[i] = to_world_space(verticesA_local[i], posA, ornA);

            if (point_in_triangle(tri.vertices, tri.normal, verticesA[i])) {
                auto pivotA = verticesA_local[i];
                auto pivotB = project_plane(verticesA[i], tri.vertices[0], tri.normal);
                auto local_distance = dot(verticesA[i] - tri.vertices[0], sep_axis);
                result.maybe_add_point({pivotA, pivotB, sep_axis, local_distance});
                ++num_edge_vert_in_tri_face;
            }
        }

        // If both vertices are inside the triangle there's nothing else to be done.
        if (num_edge_vert_in_tri_face == 2) {
            return;
        }

        // Perform edge intersections in triangle's 2D space.
        auto &tri_origin = tri.vertices[0];
        auto tangent = normalize(tri.vertices[1] - tri.vertices[0]);
        auto bitangent = cross(tri.normal, tangent); // Consequentially unit length.
        auto tri_basis = matrix3x3_columns(tangent, tri.normal, bitangent);

        auto v0A_in_tri = to_object_space(verticesA[0], tri_origin, tri_basis);
        auto v1A_in_tri = to_object_space(verticesA[1], tri_origin, tri_basis);
        auto p0 = to_vector2_xz(v0A_in_tri);
        auto p1 = to_vector2_xz(v1A_in_tri);

        for (int i = 0; i < 3; ++i) {
            // Ignore concave edges.
            if (tri.is_concave_edge[i]) {
                continue;
            }

            auto &v0 = tri.vertices[i];
            auto &v1 = tri.vertices[(i + 1) % 3];

            auto v0B_in_tri = to_object_space(v0, tri_origin, tri_basis);
            auto v1B_in_tri = to_object_space(v1, tri_origin, tri_basis);
            auto q0 = to_vector2_xz(v0B_in_tri);
            auto q1 = to_vector2_xz(v1B_in_tri);

            scalar s[2], t[2];
            auto num_points = intersect_segments(p0, p1, q0, q1, s[0], t[0], s[1], t[1]);

            for (size_t k = 0; k < num_points; ++k) {
                auto pivotA = lerp(verticesA_local[0], verticesA_local[1], s[k]);
                auto pivotB = lerp(v0, v1, t[k]);
                auto pivotA_world = lerp(verticesA[0], verticesA[1], s[k]);
                auto local_distance = dot(pivotA_world - pivotB, sep_axis);
                result.maybe_add_point({pivotA, pivotB, sep_axis, local_distance});
            }
        }
    } else if (featureA == box_feature::edge && featureB == triangle_feature::edge) {
        EDYN_ASSERT(!tri.is_concave_edge[feature_indexB]);

        auto edgeA_local = box.get_edge(feature_indexA);
        vector3 edgeA[] = {
            to_world_space(edgeA_local[0], posA, ornA),
            to_world_space(edgeA_local[1], posA, ornA)
        };

        vector3 edgeB[] = {
            tri.vertices[feature_indexB],
            tri.vertices[(feature_indexB + 1) % 3]
        };

        scalar s[2], t[2];
        vector3 pA[2], pB[2];
        size_t num_points = 0;
        closest_point_segment_segment(edgeA[0], edgeA[1], edgeB[0], edgeB[1],
                                      s[0], t[0], pA[0], pB[0], &num_points,
                                      &s[1], &t[1], &pA[1], &pB[1]);

        for (size_t i = 0; i < num_points; ++i) {
            auto pivotA = lerp(edgeA_local[0], edgeA_local[1], s[i]);
            auto pivotB = pB[i]; // Triangle already located in world space.
            result.maybe_add_point({pivotA, pivotB, sep_axis, distance});
        }
    } else if (featureA == box_feature::face && featureB == triangle_feature::vertex) {
        EDYN_ASSERT(!tri.is_concave_vertex[feature_indexB]);

        auto vertex = tri.vertices[feature_indexB];
        auto face_normal = box.get_face_normal(feature_indexA, ornA);
        auto face_vertices = box.get_face(feature_indexA, posA, ornA);

        if (point_in_polygonal_prism(face_vertices, face_normal, vertex)) {
            auto vertex_proj = vertex + sep_axis * distance;
            auto pivotA = to_object_space(vertex_proj, posA, ornA);
            auto pivotB = vertex;
            result.maybe_add_point({pivotA, pivotB, sep_axis, distance});
        }
    } else if (featureA == box_feature::vertex && featureB == triangle_feature::face) {
        auto pivotA = box.get_vertex(feature_indexA);
        auto pivotA_world = to_world_space(pivotA, posA, ornA);
        auto pivotB = pivotA_world - tri.normal * distance;
        if (point_in_triangle(tri.vertices, tri.normal, pivotB)) {
            result.maybe_add_point({pivotA, pivotB, sep_axis, distance});
        }
    }
}

}
