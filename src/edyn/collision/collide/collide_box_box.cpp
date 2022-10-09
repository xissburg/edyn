#include "edyn/collision/collide.hpp"
#include <algorithm>
#include <numeric>
#include "edyn/math/quaternion.hpp"
#include "edyn/shapes/box_shape.hpp"
#include "edyn/util/array_util.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/math/math.hpp"
#include "edyn/math/vector2_3_util.hpp"
#include "edyn/math/transform.hpp"

namespace edyn {

void collide(const box_shape &shA, const box_shape &shB,
             const collision_context &ctx, collision_result &result) {
    // Box-Box SAT. Normal of 3 faces of A, normal of 3 faces of B, 3 * 3 edge
    // cross-products. Find axis with greatest projection.
    const auto &posA = ctx.posA;
    const auto &ornA = ctx.ornA;
    const auto &posB = ctx.posB;
    const auto &ornB = ctx.ornB;
    const auto threshold = ctx.threshold;

    auto axesA = std::array<vector3, 3>{
        quaternion_x(ornA),
        quaternion_y(ornA),
        quaternion_z(ornA)
    };

    auto axesB = std::array<vector3, 3>{
        quaternion_x(ornB),
        quaternion_y(ornB),
        quaternion_z(ornB)
    };

    scalar distance = -EDYN_SCALAR_MAX;
    vector3 sep_axis;

    // A's faces.
    for (size_t i = 0; i < 3; ++i) {
        auto dir = axesA[i];
        if (dot(posA - posB, dir) < 0) {
            dir = -dir; // Point towards A.
        }

        auto projA = dot(posA, dir) - shA.half_extents[i];
        auto projB = shB.support_projection(posB, ornB, dir);
        auto dist = projA - projB;

        if (dist > distance) {
            distance = dist;
            sep_axis = dir;
        }
    }

    // B's faces.
    for (size_t i = 0; i < 3; ++i) {
        auto dir = axesB[i];
        if (dot(posA - posB, dir) < 0) {
            dir = -dir; // Point towards A.
        }

        auto projA = -shA.support_projection(posA, ornA, -dir);
        auto projB = dot(posB, dir) + shB.half_extents[i];
        auto dist = projA - projB;

        if (dist > distance) {
            distance = dist;
            sep_axis = dir;
        }
    }

    // Edge-edge.
    for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < 3; ++j) {
            auto dir = cross(axesA[i], axesB[j]);
            auto dir_len_sqr = length_sqr(dir);

            if (!(dir_len_sqr > EDYN_EPSILON)) {
                continue;
            }

            dir /= std::sqrt(dir_len_sqr);

            if (dot(posA - posB, dir) < 0) {
                // Make it point towards A.
                dir *= -1;
            }

            auto projA = -shA.support_projection(posA, ornA, -dir);
            auto projB = shB.support_projection(posB, ornB, dir);
            auto dist = projA - projB;

            if (dist > distance) {
                distance = dist;
                sep_axis = dir;
            }
        }
    }

    if (distance > threshold) {
        return;
    }

    box_feature featureA, featureB;
    size_t feature_indexA, feature_indexB;
    scalar projectionA, projectionB;

    shA.support_feature(posA, ornA, vector3_zero, -sep_axis,
                        featureA, feature_indexA, projectionA,
                        support_feature_tolerance);
    shB.support_feature(posB, ornB, vector3_zero, sep_axis,
                        featureB, feature_indexB, projectionB,
                        support_feature_tolerance);

    collision_result::collision_point point;
    point.normal = sep_axis;
    point.distance = distance;
    point.featureA = {featureA, feature_indexA};
    point.featureB = {featureB, feature_indexB};

    if (featureA == box_feature::face && featureB == box_feature::face) {
        // Face-Face.
        auto face_verticesA = shA.get_face(feature_indexA, posA, ornA);
        auto face_normalA = shA.get_face_normal(feature_indexA, ornA);
        auto face_verticesB = shB.get_face(feature_indexB, posB, ornB);
        auto face_normalB = shB.get_face_normal(feature_indexB, ornB);
        point.normal_attachment = contact_normal_attachment::normal_on_B;

        // Check for vertices of Face B contained in Face A.
        for (size_t i = 0; i < 4; ++i) {
            if (point_in_polygonal_prism(face_verticesA, face_normalA, face_verticesB[i])) {
                // Face B vertex is inside Face A.
                auto pivot_face = project_plane(face_verticesB[i], face_verticesA[0], face_normalA);
                point.pivotA = to_object_space(pivot_face, posA, ornA);
                point.pivotB = to_object_space(face_verticesB[i], posB, ornB);
                result.maybe_add_point(point);
            }
        }

        // Check for vertices of Face A contained in Face B.
        for (size_t i = 0; i < 4; ++i) {
            if (point_in_polygonal_prism(face_verticesB, face_normalB, face_verticesA[i])) {
                // Face A vertex is inside Face B.
                auto pivot_face = project_plane(face_verticesA[i], face_verticesB[0], face_normalB);
                point.pivotA = to_object_space(face_verticesA[i], posA, ornA);
                point.pivotB = to_object_space(pivot_face, posB, ornB);
                result.maybe_add_point(point);
            }
        }

        // If not all vertices are contained in a face, perform edge intersection tests.
        if (result.num_points < 4) {
            auto face_center = shA.get_face_center(feature_indexA, posA, ornA);
            auto face_basis = shA.get_face_basis(feature_indexA, ornA);
            auto half_extents = shA.get_face_half_extents(feature_indexA);

            for (size_t j = 0; j < 4; ++j) {
                auto b0_world = face_verticesB[j];
                auto b1_world = face_verticesB[(j + 1) % 4];
                auto b0 = to_object_space(b0_world, face_center, face_basis);
                auto b1 = to_object_space(b1_world, face_center, face_basis);
                auto p0 = to_vector2_xz(b0);
                auto p1 = to_vector2_xz(b1);

                scalar s[2];
                auto num_points = intersect_line_aabb(p0, p1, -half_extents, half_extents, s[0], s[1]);
                for (size_t k = 0; k < num_points; ++k) {
                    if (s[k] < 0 || s[k] > 1) continue;

                    auto q1 = lerp(b0_world, b1_world, s[k]);
                    auto q0 = project_plane(q1, face_center, face_normalA);
                    point.pivotA = to_object_space(q0, posA, ornA);
                    point.pivotB = to_object_space(q1, posB, ornB);
                    result.maybe_add_point(point);
                }
            }
        }
    } else if ((featureA == box_feature::face && featureB == box_feature::edge) ||
               (featureB == box_feature::face && featureA == box_feature::edge)) {
        // Face vs Edge.
        const auto is_faceA = featureA == box_feature::face;

        auto face_normal = is_faceA ? shA.get_face_normal(feature_indexA, ornA) :
                                      shB.get_face_normal(feature_indexB, ornB);
        auto face_vertices = is_faceA ? shA.get_face(feature_indexA, posA, ornA) :
                                        shB.get_face(feature_indexB, posB, ornB);
        auto edge_vertices = is_faceA ? shB.get_edge(feature_indexB, posB, ornB) :
                                        shA.get_edge(feature_indexA, posA, ornA);
        point.normal_attachment = is_faceA ?
            contact_normal_attachment::normal_on_A :
            contact_normal_attachment::normal_on_B;

        // Check if edge vertices are inside face.
        for (int i = 0; i < 2; ++i) {
            if (point_in_polygonal_prism(face_vertices, face_normal, edge_vertices[i])) {
                // Edge's vertex is inside face.
                auto pivot_face = project_plane(edge_vertices[i], face_vertices[0], face_normal);
                point.pivotA = is_faceA ? to_object_space(pivot_face, posA, ornA) :
                                          to_object_space(edge_vertices[i], posA, ornA);
                point.pivotB = is_faceA ? to_object_space(edge_vertices[i], posB, ornB) :
                                          to_object_space(pivot_face, posB, ornB);
                result.add_point(point);
            }
        }

        // If both vertices are not inside the face then perform edge intersection tests.
        if (result.num_points < 2) {
            auto face_center = is_faceA ? shA.get_face_center(feature_indexA, posA, ornA) :
                                          shB.get_face_center(feature_indexB, posB, ornB);
            auto face_basis = is_faceA ? shA.get_face_basis(feature_indexA, ornA) :
                                         shB.get_face_basis(feature_indexB, ornB);
            auto half_extents = is_faceA ? shA.get_face_half_extents(feature_indexA) :
                                           shB.get_face_half_extents(feature_indexB);

            auto e0 = to_object_space(edge_vertices[0], face_center, face_basis);
            auto e1 = to_object_space(edge_vertices[1], face_center, face_basis);
            auto p0 = to_vector2_xz(e0);
            auto p1 = to_vector2_xz(e1);

            scalar s[2];
            auto num_points = intersect_line_aabb(p0, p1, -half_extents, half_extents, s[0], s[1]);

            for (size_t i = 0; i < num_points; ++i) {
                if (s[i] < 0 || s[i] > 1) continue;

                auto edge_pivot = lerp(edge_vertices[0], edge_vertices[1], s[i]);
                auto face_pivot = project_plane(edge_pivot, face_center, sep_axis);
                point.pivotA = to_object_space(is_faceA ? face_pivot : edge_pivot, posA, ornA);
                point.pivotB = to_object_space(is_faceA ? edge_pivot : face_pivot, posB, ornB);
                result.add_point(point);
            }
        }
    } else if (featureA == box_feature::edge && featureB == box_feature::edge) {
        // Edge-Edge.
        scalar s[2], t[2];
        vector3 p0[2], p1[2];
        size_t num_points = 0;
        auto edgeA = shA.get_edge(feature_indexA, posA, ornA);
        auto edgeB = shB.get_edge(feature_indexB, posB, ornB);
        closest_point_segment_segment(edgeA[0], edgeA[1], edgeB[0], edgeB[1],
                                      s[0], t[0], p0[0], p1[0], &num_points,
                                      &s[1], &t[1], &p0[1], &p1[1]);
        point.normal_attachment = contact_normal_attachment::none;

        for (size_t i = 0; i < num_points; ++i) {
            point.pivotA = to_object_space(p0[i], posA, ornA);
            point.pivotB = to_object_space(p1[i], posB, ornB);
            result.add_point(point);
        }
    } else if (featureA == box_feature::face && featureB == box_feature::vertex) {
        // Face A, Vertex B.
        point.pivotB = shB.get_vertex(feature_indexB);
        point.pivotA = to_world_space(point.pivotB, posB, ornB) + sep_axis * distance;
        point.pivotA = to_object_space(point.pivotA, posA, ornA);
        point.normal_attachment = contact_normal_attachment::normal_on_A;
        result.add_point(point);
    } else if (featureB == box_feature::face && featureA == box_feature::vertex) {
        // Face B, Vertex A.
        point.pivotA = shA.get_vertex(feature_indexA);
        point.pivotB = to_world_space(point.pivotA, posA, ornA) - sep_axis * distance;
        point.pivotB = to_object_space(point.pivotB, posB, ornB);
        point.normal_attachment = contact_normal_attachment::normal_on_B;
        result.add_point(point);
    }
}

}
