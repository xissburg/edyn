#include "edyn/collision/collide.hpp"
#include "edyn/config/constants.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/math/math.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/math/vector2_3_util.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/math/transform.hpp"
#include "edyn/util/shape_util.hpp"
#include "edyn/math/triangle.hpp"

namespace edyn {

static void collide_box_triangle(
    const box_shape &box, const triangle_mesh &mesh, size_t tri_idx,
    const std::array<vector3, 3> &box_axes,
    const collision_context &ctx, collision_result &result) {

    const auto &posA = ctx.posA;
    const auto &ornA = ctx.ornA;
    const auto tri_vertices = mesh.get_triangle_vertices(tri_idx);
    const auto tri_normal = mesh.get_triangle_normal(tri_idx);
    const auto tri_center = average(tri_vertices);

    auto distance = -EDYN_SCALAR_MAX;
    auto sep_axis = vector3_zero;

    // Check if the given direction is the best so far and set it if so.
    auto test_direction = [&](vector3 dir) {
        auto projA = -box.support_projection(posA, ornA, -dir);
        auto projB = get_triangle_support_projection(tri_vertices, dir);
        auto dist = projA - projB;

        if (dist > distance) {
            distance = dist;
            sep_axis = dir;
        }
    };

    // Triangle face normal.
    {
        auto proj = box.support_projection(posA, ornA, -tri_normal);
        // No need to check if greater than `distance` since this is the first.
        distance = -(proj + dot(tri_vertices[0], tri_normal));
        sep_axis = tri_normal;
    }

    // Box faces.
    for (size_t i = 0; i < 6; ++i) {
        auto j = i % 3;
        auto dir = box_axes[j] * to_sign(i < 3);

        auto projA = -(dot(posA, -dir) + box.half_extents[j]);
        auto projB = get_triangle_support_projection(tri_vertices, dir);
        auto dist = projA - projB;

        if (dist > distance) {
            distance = dist;
            sep_axis = dir;
        }
    }

    // Edges.
    for (size_t i = 0; i < 3; ++i) {
        auto &axisA = box_axes[i];

        for (size_t j = 0; j < 3; ++j) {
            auto axisB = tri_vertices[(j + 1) % 3] - tri_vertices[j];
            auto dir = cross(axisA, axisB);

            if (!try_normalize(dir)) {
                continue;
            }

            // Choose the direction that points towards the vector that
            // goes from the center of the triangle to the center of the box,
            // (i.e. has a positive dot product with it), or...
            if (dot(posA - tri_center, dir) < 0) {
                dir *= -1;
            }

            test_direction(dir);

            // ...a more accurate though more expensive approach is to try both
            // directions and pick whichever gives the least amount of penetration.
            //test_direction(dir);
            //test_direction(-dir);
        }
    }

    if (distance > ctx.threshold) {
        return;
    }

    triangle_feature tri_feature;
    size_t tri_feature_index;
    scalar proj_tri;
    get_triangle_support_feature(tri_vertices, vector3_zero, sep_axis,
                                 tri_feature, tri_feature_index,
                                 proj_tri, support_feature_tolerance);

    // Adjust separating axis to keep in the closest feature's Voronoi region.
    auto new_sep_axis = clip_triangle_separating_axis(sep_axis, mesh, tri_idx,
                                                      tri_vertices, tri_normal,
                                                      tri_feature, tri_feature_index);

    if (new_sep_axis != sep_axis) {
        sep_axis = new_sep_axis;
        get_triangle_support_feature(tri_vertices, vector3_zero, sep_axis,
                                     tri_feature, tri_feature_index,
                                     proj_tri, support_feature_tolerance);
    }

    box_feature box_feature;
    size_t box_feature_index;
    scalar proj_box;
    box.support_feature(posA, ornA, vector3_zero, -sep_axis,
                        box_feature, box_feature_index, proj_box,
                        support_feature_tolerance);

    distance = -proj_box - proj_tri;

    if (distance > ctx.threshold) {
        return;
    }

    if (-distance > mesh.get_thickness()) {
        return;
    }

    collision_result::collision_point point;
    point.normal = sep_axis;
    point.distance = distance;
    point.featureA = {box_feature, box_feature_index};
    point.featureB = {tri_feature};
    point.featureB->index = get_triangle_mesh_feature_index(mesh, tri_idx, tri_feature, tri_feature_index);

    if (box_feature == box_feature::face && tri_feature == triangle_feature::face) {
        auto normalA = box.get_face_normal(box_feature_index, ornA);
        auto verticesA_local = box.get_face(box_feature_index);
        std::array<vector3, 4> verticesA;
        for (int i = 0; i < 4; ++i) {
            verticesA[i] = to_world_space(verticesA_local[i], posA, ornA);
        }

        // Check for triangle vertices inside box face.
        size_t num_tri_vert_in_box_face = 0;
        point.normal_attachment = contact_normal_attachment::normal_on_B;

        for (int i = 0; i < 3; ++i) {
            if (point_in_polygonal_prism(verticesA, normalA, tri_vertices[i])) {
                // Triangle vertex is inside box face.
                auto pivot_on_face = project_plane(tri_vertices[i], verticesA[0], normalA);
                point.pivotA = to_object_space(pivot_on_face, posA, ornA);
                point.pivotB = tri_vertices[i];
                point.distance = dot(pivot_on_face - point.pivotB, sep_axis);
                result.maybe_add_point(point);
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
            if (point_in_triangle(tri_vertices, tri_normal, verticesA[i])) {
                point.pivotA = verticesA_local[i];
                point.pivotB = project_plane(verticesA[i], tri_vertices[0], tri_normal);
                point.distance = dot(verticesA[i] - tri_vertices[0], sep_axis);
                result.maybe_add_point(point);
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
            auto &b0 = tri_vertices[i];
            auto &b1 = tri_vertices[(i + 1) % 3];

            // Convert this into a 2D segment intersection problem in the box' space.
            auto b0_in_A = to_object_space(b0, posA, ornA);
            auto b1_in_A = to_object_space(b1, posA, ornA);

            vector2 half_extents;
            vector2 p0, p1;

            // Pick the planar coordinates based on the face index.
            if (box_feature_index == 0 || box_feature_index == 1) { // X face
                half_extents = to_vector2_zy(box.half_extents);
                p0 = to_vector2_zy(b0_in_A); p1 = to_vector2_zy(b1_in_A);
            } else if (box_feature_index == 2 || box_feature_index == 3) { // Y face
                half_extents = to_vector2_xz(box.half_extents);
                p0 = to_vector2_xz(b0_in_A); p1 = to_vector2_xz(b1_in_A);
            } else { // if (box_feature_index == 4 || box_feature_index == 5) { // Z face
                half_extents = to_vector2_xy(box.half_extents);
                p0 = to_vector2_xy(b0_in_A); p1 = to_vector2_xy(b1_in_A);
            }

            scalar s[2];
            auto num_points = intersect_line_aabb(p0, p1, -half_extents, half_extents, s[0], s[1]);

            for (size_t k = 0; k < num_points; ++k) {
                if (s[k] < 0 || s[k] > 1) continue;

                point.pivotB = lerp(b0, b1, s[k]);
                auto pivotA_world = project_plane(point.pivotB, verticesA[0], normalA);
                point.pivotA = to_object_space(pivotA_world, posA, ornA);
                point.distance = dot(pivotA_world - point.pivotB, sep_axis);
                result.maybe_add_point(point);
            }
        }
    } else if (box_feature == box_feature::face && tri_feature == triangle_feature::edge) {
        EDYN_ASSERT(mesh.is_convex_edge(mesh.get_face_edge_index(tri_idx, tri_feature_index)));

        auto normalA = box.get_face_normal(box_feature_index, ornA);
        auto verticesA_local = box.get_face(box_feature_index);
        std::array<vector3, 4> verticesA;
        for (int i = 0; i < 4; ++i) {
            verticesA[i] = to_world_space(verticesA_local[i], posA, ornA);
        }

        // Check if edge vertices are inside box face.
        vector3 edge_vertices[] = {tri_vertices[tri_feature_index],
                                   tri_vertices[(tri_feature_index + 1) % 3]};
        size_t num_edge_vert_in_box_face = 0;
        point.normal_attachment = contact_normal_attachment::normal_on_A;

        for (int i = 0; i < 2; ++i) {
            if (point_in_polygonal_prism(verticesA, normalA, edge_vertices[i])) {
                // Edge's vertex is inside face.
                auto pivot_on_face = project_plane(edge_vertices[i], verticesA[0], normalA);
                point.pivotA = to_object_space(pivot_on_face, posA, ornA);
                point.pivotB = edge_vertices[i];
                point.distance = dot(pivot_on_face - point.pivotB, sep_axis);
                result.maybe_add_point(point);
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

        if (box_feature_index == 0 || box_feature_index == 1) { // X face
            half_extents = to_vector2_zy(box.half_extents);
            p0 = to_vector2_zy(e0_in_A); p1 = to_vector2_zy(e1_in_A);
        } else if (box_feature_index == 2 || box_feature_index == 3) { // Y face
            half_extents = to_vector2_xz(box.half_extents);
            p0 = to_vector2_xz(e0_in_A); p1 = to_vector2_xz(e1_in_A);
        } else { // if (box_feature_index == 4 || box_feature_index == 5) { // Z face
            half_extents = to_vector2_xy(box.half_extents);
            p0 = to_vector2_xy(e0_in_A); p1 = to_vector2_xy(e1_in_A);
        }

        scalar s[2];
        auto num_points = intersect_line_aabb(p0, p1, -half_extents, half_extents, s[0], s[1]);

        for (size_t k = 0; k < num_points; ++k) {
            if (s[k] < 0 || s[k] > 1) continue;

            point.pivotB = lerp(edge_vertices[0], edge_vertices[1], s[k]);
            auto pivotA_world = project_plane(point.pivotB, verticesA[0], normalA);
            point.pivotA = to_object_space(pivotA_world, posA, ornA);
            point.distance = dot(pivotA_world - point.pivotB, sep_axis);
            result.maybe_add_point(point);
        }
    } else if (box_feature == box_feature::edge && tri_feature == triangle_feature::face) {
        // Check if edge vertices are inside triangle face.
        auto verticesA_local = box.get_edge(box_feature_index);
        std::array<vector3, 2> verticesA;
        size_t num_edge_vert_in_tri_face = 0;
        point.normal_attachment = contact_normal_attachment::normal_on_B;

        for (size_t i = 0; i < 2; ++i) {
            verticesA[i] = to_world_space(verticesA_local[i], posA, ornA);

            if (point_in_triangle(tri_vertices, tri_normal, verticesA[i])) {
                point.pivotA = verticesA_local[i];
                point.pivotB = project_plane(verticesA[i], tri_vertices[0], tri_normal);
                point.distance = dot(verticesA[i] - tri_vertices[0], sep_axis);
                result.maybe_add_point(point);
                ++num_edge_vert_in_tri_face;
            }
        }

        // If both vertices are inside the triangle there's nothing else to be done.
        if (num_edge_vert_in_tri_face == 2) {
            return;
        }

        // Perform edge intersections in triangle's 2D space.
        auto &tri_origin = tri_vertices[0];
        auto tangent = normalize(tri_vertices[1] - tri_vertices[0]);
        auto bitangent = cross(tri_normal, tangent); // Consequentially unit length.
        auto tri_basis = matrix3x3_columns(tangent, tri_normal, bitangent);

        auto v0A_in_tri = to_object_space(verticesA[0], tri_origin, tri_basis);
        auto v1A_in_tri = to_object_space(verticesA[1], tri_origin, tri_basis);
        auto p0 = to_vector2_xz(v0A_in_tri);
        auto p1 = to_vector2_xz(v1A_in_tri);

        for (int i = 0; i < 3; ++i) {
            auto &v0 = tri_vertices[i];
            auto &v1 = tri_vertices[(i + 1) % 3];

            auto v0B_in_tri = to_object_space(v0, tri_origin, tri_basis);
            auto v1B_in_tri = to_object_space(v1, tri_origin, tri_basis);
            auto q0 = to_vector2_xz(v0B_in_tri);
            auto q1 = to_vector2_xz(v1B_in_tri);

            scalar s[2], t[2];
            auto num_points = intersect_segments(p0, p1, q0, q1, s[0], t[0], s[1], t[1]);

            for (size_t k = 0; k < num_points; ++k) {
                point.pivotA = lerp(verticesA_local[0], verticesA_local[1], s[k]);
                point.pivotB = lerp(v0, v1, t[k]);
                auto pivotA_world = lerp(verticesA[0], verticesA[1], s[k]);
                point.distance = dot(pivotA_world - point.pivotB, sep_axis);
                result.maybe_add_point(point);
            }
        }
    } else if (box_feature == box_feature::edge && tri_feature == triangle_feature::edge) {
        EDYN_ASSERT(mesh.is_convex_edge(mesh.get_face_edge_index(tri_idx, tri_feature_index)));

        auto edgeA_local = box.get_edge(box_feature_index);
        vector3 edgeA[] = {
            to_world_space(edgeA_local[0], posA, ornA),
            to_world_space(edgeA_local[1], posA, ornA)
        };

        vector3 edgeB[] = {
            tri_vertices[tri_feature_index],
            tri_vertices[(tri_feature_index + 1) % 3]
        };

        point.normal_attachment = contact_normal_attachment::none;

        scalar s[2], t[2];
        vector3 pA[2], pB[2];
        size_t num_points = 0;
        closest_point_segment_segment(edgeA[0], edgeA[1], edgeB[0], edgeB[1],
                                      s[0], t[0], pA[0], pB[0], &num_points,
                                      &s[1], &t[1], &pA[1], &pB[1]);

        for (size_t i = 0; i < num_points; ++i) {
            if (!(s[i] > 0 && s[i] < 1 && t[i] > 0 && t[i] < 1)) continue;

            point.pivotA = lerp(edgeA_local[0], edgeA_local[1], s[i]);
            point.pivotB = pB[i]; // Triangle already located in world space.
            result.maybe_add_point(point);
        }
    } else if (box_feature == box_feature::face && tri_feature == triangle_feature::vertex) {
        auto vertex = tri_vertices[tri_feature_index];
        auto face_normal = box.get_face_normal(box_feature_index, ornA);
        auto face_vertices = box.get_face(box_feature_index, posA, ornA);
        point.normal_attachment = contact_normal_attachment::normal_on_A;

        if (point_in_polygonal_prism(face_vertices, face_normal, vertex)) {
            auto vertex_proj = vertex + sep_axis * distance;
            point.pivotA = to_object_space(vertex_proj, posA, ornA);
            point.pivotB = vertex;
            result.maybe_add_point(point);
        }
    } else if (box_feature == box_feature::vertex && tri_feature == triangle_feature::face) {
        auto pivotA = box.get_vertex(box_feature_index);
        auto pivotA_world = to_world_space(pivotA, posA, ornA);
        point.normal_attachment = contact_normal_attachment::normal_on_B;

        if (point_in_triangle(tri_vertices, tri_normal, pivotA_world)) {
            point.pivotA = pivotA;
            point.pivotB = pivotA_world - tri_normal * distance;
            result.maybe_add_point(point);
        }
    }
}

void collide(const box_shape &box, const triangle_mesh &mesh,
             const collision_context &ctx, collision_result &result) {
    const auto box_axes = std::array<vector3, 3> {
        quaternion_x(ctx.ornA),
        quaternion_y(ctx.ornA),
        quaternion_z(ctx.ornA)
    };

    const auto inset = vector3_one * -contact_breaking_threshold;
    const auto visit_aabb = ctx.aabbA.inset(inset);

    mesh.visit_triangles(visit_aabb, [&](auto tri_idx) {
        collide_box_triangle(box, mesh, tri_idx, box_axes, ctx, result);
    });
}

}
