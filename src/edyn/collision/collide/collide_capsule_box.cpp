#include "edyn/collision/collide.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/math/vector2_3_util.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/math/transform.hpp"
#include "edyn/shapes/box_shape.hpp"
#include "edyn/shapes/capsule_shape.hpp"
#include "edyn/util/shape_util.hpp"
#include "edyn/math/math.hpp"

namespace edyn {

void collide(const capsule_shape &shA, const box_shape &shB,
             const collision_context &ctx, collision_result &result) {
    const auto posA = vector3_zero;
    const auto &ornA = ctx.ornA;
    const auto posB = ctx.posB - ctx.posA;
    const auto &ornB = ctx.ornB;
    const auto threshold = ctx.threshold;

    auto capsule_vertices = shA.get_vertices(posA, ornA);

    const auto box_axes = std::array<vector3, 3>{
        quaternion_x(ornB),
        quaternion_y(ornB),
        quaternion_z(ornB)
    };

    scalar distance = -EDYN_SCALAR_MAX;
    scalar projection_box = -EDYN_SCALAR_MAX;
    auto sep_axis = vector3_zero;

    // Face normals of box.
    for (size_t i = 0; i < 3; ++i) {
        auto dir = box_axes[i];
        if (dot(posA - posB, dir) < 0) {
            dir = -dir; // Point towards capsule.
        }

        auto projA = -capsule_support_projection(capsule_vertices, shA.radius, -dir);
        auto projB = dot(posB, dir) + shB.half_extents[i];
        auto dist = projA - projB;

        if (dist > distance) {
            distance = dist;
            projection_box = projB;
            sep_axis = dir;
        }
    }

    // Box edges vs capsule edge.
    for (size_t i = 0; i < get_box_num_features(box_feature::edge); ++i) {
        auto [vertexA0, vertexA1] = shB.get_edge(i, posB, ornB);
        scalar s, t;
        vector3 closestA, closestB;
        closest_point_segment_segment(vertexA0, vertexA1,
                                      capsule_vertices[0], capsule_vertices[1],
                                      s, t, closestA, closestB);
        auto dir = closestA - closestB;

        if (!try_normalize(dir)) {
            continue;
        }

        if (dot(posA - posB, dir) < 0) {
            dir *= -1; // Make it point towards A.
        }

        auto projA = -capsule_support_projection(capsule_vertices, shA.radius, -dir);
        auto projB = shB.support_projection(posB, ornB, dir);
        auto dist = projA - projB;

        if (dist > distance) {
            distance = dist;
            projection_box = projB;
            sep_axis = dir;
        }
    }

    if (distance > threshold) {
        return;
    }

    scalar proj_capsule_vertices[] = {
        dot(capsule_vertices[0], sep_axis),
        dot(capsule_vertices[1], sep_axis)
    };

    capsule_feature featureA;
    size_t feature_indexA = {};
    auto is_capsule_edge = std::abs(proj_capsule_vertices[0] -
                                    proj_capsule_vertices[1]) < support_feature_tolerance;

    if (is_capsule_edge) {
        featureA = capsule_feature::side;
    } else {
        featureA = capsule_feature::hemisphere;
        feature_indexA = proj_capsule_vertices[0] < proj_capsule_vertices[1] ? 0 : 1;
    }

    auto contact_origin_box = sep_axis * projection_box;
    scalar feature_distanceB;
    box_feature featureB;
    size_t feature_indexB;
    shB.support_feature(posB, ornB, contact_origin_box, sep_axis,
                        featureB, feature_indexB,
                        feature_distanceB, support_feature_tolerance);

    collision_result::collision_point point;
    point.normal = sep_axis;
    point.distance = distance;
    point.featureA = {featureA, feature_indexA};
    point.featureB = {featureB, feature_indexB};

    switch (featureB) {
    case box_feature::face: {
        auto face_vertices = shB.get_face(feature_indexB, posB, ornB);
        point.normal_attachment = contact_normal_attachment::normal_on_B;

        if (is_capsule_edge) {
            // Check if vertices of the capsule on the contact plane are inside
            // the box face.
            for (auto &pointA : capsule_vertices) {
                if (point_in_polygonal_prism(face_vertices, sep_axis, pointA)) {
                    point.pivotA = to_object_space(pointA - sep_axis * shA.radius, posA, ornA);
                    auto pivotB_world = project_plane(pointA, contact_origin_box, sep_axis);
                    point.pivotB = to_object_space(pivotB_world, posB, ornB);
                    result.add_point(point);
                }
            }

            // Do not continue if there are already 2 points in the result, which means
            // both vertices of the capsule are contained in the face.
            if (result.num_points == 2) {
                return;
            }

            // Check if the capsule edge intersects the edges of the box face.
            auto face_center = shB.get_face_center(feature_indexB, posB, ornB);
            auto face_basis = shB.get_face_basis(feature_indexB, ornB);
            auto half_extents = shB.get_face_half_extents(feature_indexB);

            auto p0 = to_vector2_xz(to_object_space(capsule_vertices[0], face_center, face_basis));
            auto p1 = to_vector2_xz(to_object_space(capsule_vertices[1], face_center, face_basis));

            scalar s[2];
            auto num_points = intersect_line_aabb(p0, p1, -half_extents, half_extents, s[0], s[1]);

            for (size_t i = 0; i < num_points; ++i) {
                if (s[i] < 0 || s[i] > 1) continue;

                auto edge_pivot = lerp(capsule_vertices[0], capsule_vertices[1], s[i]);
                auto face_pivot = project_plane(edge_pivot, face_center, sep_axis);
                point.pivotA = to_object_space(edge_pivot - sep_axis * shA.radius, posA, ornA);
                point.pivotB = to_object_space(face_pivot, posB, ornB);
                result.add_point(point);
            }
        } else {
            // Capsule edge vs box face.
            auto &closest_capsule_vertex = proj_capsule_vertices[0] < proj_capsule_vertices[1] ?
                                           capsule_vertices[0] : capsule_vertices[1];
            auto pivotA_world = closest_capsule_vertex - sep_axis * shA.radius;
            auto pivotB_world = project_plane(pivotA_world, contact_origin_box, sep_axis);
            point.pivotA = to_object_space(pivotA_world, posA, ornA);
            point.pivotB = to_object_space(pivotB_world, posB, ornB);
            result.add_point(point);
        }
        break;
    }
    case box_feature::edge: {
        auto edge_vertices = shB.get_edge(feature_indexB, posB, ornB);
        point.normal_attachment = contact_normal_attachment::none;

        if (is_capsule_edge) {
            scalar s[2], t[2];
            vector3 closestA[2], closestB[2];
            size_t num_points;
            closest_point_segment_segment(capsule_vertices[0], capsule_vertices[1],
                                          edge_vertices[0], edge_vertices[1],
                                          s[0], t[0], closestA[0], closestB[0], &num_points,
                                          &s[1], &t[1], &closestA[1], &closestB[1]);

            for (size_t i = 0; i < num_points; ++i) {
                auto pivotA_world = closestA[i] - sep_axis * shA.radius;
                auto pivotB_world = closestB[i];
                point.pivotA = to_object_space(pivotA_world, posA, ornA);
                point.pivotB = to_object_space(pivotB_world, posB, ornB);
                result.add_point(point);
            }
        } else {
            // Capsule vertex against box edge.
            auto &closest_capsule_vertex = proj_capsule_vertices[0] < proj_capsule_vertices[1] ?
                                           capsule_vertices[0] : capsule_vertices[1];
            auto edge_dir = edge_vertices[1] - edge_vertices[0];
            vector3 pivotB_world; scalar t;
            closest_point_line(edge_vertices[0], edge_dir, closest_capsule_vertex, t, pivotB_world);
            point.pivotB = to_object_space(pivotB_world, posB, ornB);
            point.pivotA = to_object_space(closest_capsule_vertex - sep_axis * shA.radius, posA, ornA);
            result.add_point(point);
        }
        break;
    }
    case box_feature::vertex: {
        point.pivotB = shB.get_vertex(feature_indexB);
        auto pivotB_world = to_world_space(point.pivotB, posB, ornB);
        auto pivotA_world = pivotB_world + sep_axis * distance;
        point.pivotA = to_object_space(pivotA_world, posA, ornA);
        point.normal_attachment = contact_normal_attachment::none;
        result.add_point(point);
    }
    }
}

void collide(const box_shape &shA, const capsule_shape &shB,
             const collision_context &ctx, collision_result &result) {
    swap_collide(shA, shB, ctx, result);
}

}
