#include "edyn/collision/collide.hpp"
#include "edyn/math/vector2_3_util.hpp"
#include "edyn/math/transform.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/math/math.hpp"
#include "edyn/util/shape_util.hpp"

namespace edyn {

void collide(const capsule_shape &shA, const cylinder_shape &shB,
             const collision_context &ctx, collision_result &result) {
    const auto posA = vector3_zero;
    const auto &ornA = ctx.ornA;
    const auto posB = ctx.posB - ctx.posA;
    const auto &ornB = ctx.ornB;

    auto capsule_vertices = shA.get_vertices(posA, ornA);
    auto cylinder_vertices = shB.get_vertices(posB, ornB);

    auto cap_axis = normalize(capsule_vertices[1] - capsule_vertices[0]);
    auto cyl_axis = normalize(cylinder_vertices[1] - cylinder_vertices[0]);

    scalar distance = -EDYN_SCALAR_MAX;
    auto sep_axis = vector3_zero;

    // Cylinder cap faces.
    {
        auto dir = cyl_axis;

        if (dot(posA - posB, dir) < 0) {
            dir *= -1; // Make dir point towards A.
        }

        auto projA = -capsule_support_projection(capsule_vertices, shA.radius, -dir);
        auto projB = dot(posB, dir) + shB.half_length;
        auto dist = projA - projB;

        if (dist > distance) {
            distance = dist;
            sep_axis = dir;
        }
    }

    // Cylinder edge vs capsule edge.
    {
        auto dir = cross(cyl_axis, cap_axis);

        if (try_normalize(dir)) {
            if (dot(posA - posB, dir) < 0) {
                dir *= -1; // Make dir point towards A.
            }

            auto projA = dot(posA, dir) - shA.radius;
            auto projB = dot(posB, dir) + shB.radius;
            auto dist = projA - projB;

            if (dist > distance) {
                distance = dist;
                sep_axis = dir;
            }
        }
    }

    // Cylinder edge vs capsule vertices.
    for (auto &vertex : capsule_vertices) {
        vector3 closest; scalar t;
        closest_point_line(posB, cyl_axis, vertex, t, closest);
        auto dir = vertex - closest; // Points towards A.

        if (!try_normalize(dir)) {
            continue;
        }

        auto projA = -capsule_support_projection(capsule_vertices, shA.radius, -dir);
        auto projB = shB.support_projection(posB, ornB, dir);
        auto dist = projA - projB;

        if (dist > distance) {
            distance = dist;
            sep_axis = dir;
        }
    }

    // Cylinder caps vs capsule edge.
    for (size_t i = 0; i < 2; ++i) {
        scalar s[2];
        size_t num_points;
        vector3 closest_circle[2], closest_line[2];
        vector3 dir;

        closest_point_circle_line(cylinder_vertices[i], ornB, shB.radius,
                                  capsule_vertices[0], capsule_vertices[1], num_points,
                                  s[0], closest_circle[0], closest_line[0],
                                  s[1], closest_circle[1], closest_line[1], dir);

        if (dot(posA - posB, dir) < 0) {
            dir *= -1; // Make it point towards A.
        }

        auto projA = -capsule_support_projection(capsule_vertices, shA.radius, -dir);
        auto projB = shB.support_projection(posB, ornB, dir);
        auto dist = projA - projB;

        if (dist > distance) {
            distance = dist;
            sep_axis = dir;
        }
    }

    // Cylinder caps vs capsule vertices.
    for (size_t i = 0; i < 2; ++i) {
        for (size_t j = 0; j < 2; ++j) {
            auto &vertex = capsule_vertices[j];
            vector3 closest;
            closest_point_disc(cylinder_vertices[i], ornB, shB.radius, vertex, closest);
            auto dir = closest - vertex;

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
                sep_axis = dir;
            }
        }
    }

    if (distance > ctx.threshold) {
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

    cylinder_feature featureB;
    size_t feature_indexB;
    shB.support_feature(posB, ornB, sep_axis, featureB, feature_indexB,
                        support_feature_tolerance);

    collision_result::collision_point point;
    point.normal = sep_axis;
    point.distance = distance;
    point.featureA = {featureA, feature_indexA};
    point.featureB = {featureB, feature_indexB};

    switch (featureB) {
    case cylinder_feature::face: {
        point.normal_attachment = contact_normal_attachment::normal_on_B;

        if (is_capsule_edge) {
            // Check if the capsule edge intersects the circular cap.
            auto v0 = to_object_space(capsule_vertices[0], posB, ornB);
            auto v1 = to_object_space(capsule_vertices[1], posB, ornB);
            scalar s[2];

            auto num_points = intersect_line_circle(to_vector2_zy(v0), to_vector2_zy(v1),
                                                    shB.radius, s[0], s[1]);

            for (size_t i = 0; i < num_points; ++i) {
                auto t = clamp_unit(s[i]);
                auto pivotA_world = lerp(capsule_vertices[0], capsule_vertices[1], t) - sep_axis * shA.radius;
                auto pivotB_world = project_plane(pivotA_world, cylinder_vertices[feature_indexB], sep_axis);
                point.pivotA = to_object_space(pivotA_world, posA, ornA);
                point.pivotB = to_object_space(pivotB_world, posB, ornB);
                point.distance = dot(pivotA_world - pivotB_world, sep_axis);
                result.add_point(point);
            }
        } else {
            // Cylinder cap face against capsule vertex.
            auto &closest_capsule_vertex = proj_capsule_vertices[0] < proj_capsule_vertices[1] ?
                                           capsule_vertices[0] : capsule_vertices[1];
            auto pivotA_world = closest_capsule_vertex - sep_axis * shA.radius;
            auto pivotB_world = project_plane(closest_capsule_vertex, cylinder_vertices[feature_indexB], sep_axis);
            point.pivotA = to_object_space(pivotA_world, posA, ornA);
            point.pivotB = to_object_space(pivotB_world, posB, ornB);
            result.add_point(point);
        }
        break;
    }
    case cylinder_feature::side_edge: {
        point.normal_attachment = contact_normal_attachment::none;
        scalar s[2], t[2];
        vector3 closest_capsule[2], closest_cylinder[2];
        size_t num_points;

        closest_point_segment_segment(
            capsule_vertices[0], capsule_vertices[1],
            cylinder_vertices[0], cylinder_vertices[1],
            s[0], t[0], closest_capsule[0], closest_cylinder[0], &num_points,
            &s[1], &t[1], &closest_capsule[1], &closest_cylinder[1]);

        for (size_t i = 0; i < num_points; ++i) {
            point.pivotA = to_object_space(closest_capsule[i] - sep_axis * shA.radius, posA, ornA);
            point.pivotB = to_object_space(closest_cylinder[i] + sep_axis * shB.radius, posB, ornB);
            result.add_point(point);
        }

        break;
    }
    case cylinder_feature::cap_edge: {
        point.normal_attachment = contact_normal_attachment::none;
        auto supportB = shB.support_point(posB, ornB, sep_axis);
        point.pivotB = to_object_space(supportB, posB, ornB);
        point.pivotA = to_object_space(supportB + sep_axis * distance, posA, ornA);
        result.add_point(point);
        break;
    }
    }
}

void collide(const cylinder_shape &shA, const capsule_shape &shB,
             const collision_context &ctx, collision_result &result) {
    swap_collide(shA, shB, ctx, result);
}

}
