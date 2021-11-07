#include "edyn/collision/collide.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/math/math.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/math/vector2.hpp"
#include "edyn/math/vector2_3_util.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/math/transform.hpp"
#include <array>

namespace edyn {

void collide(const cylinder_shape &shA, const cylinder_shape &shB,
             const collision_context &ctx, collision_result &result) {
    // Cylinder-cylinder SAT.
    const auto &posA = ctx.posA;
    const auto &ornA = ctx.ornA;
    const auto &posB = ctx.posB;
    const auto &ornB = ctx.ornB;

    const auto axisA = quaternion_x(ornA);
    const auto axisB = quaternion_x(ornB);

    const auto verticesA = std::array<vector3, 2>{
        posA + axisA * shA.half_length,
        posA - axisA * shA.half_length
    };

    const auto verticesB = std::array<vector3, 2>{
        posB + axisB * shB.half_length,
        posB - axisB * shB.half_length
    };

    vector3 sep_axis;
    scalar distance = -EDYN_SCALAR_MAX;

    // A's faces.
    {
        vector3 dir = axisA;

        if (dot(posA - posB, dir) < 0) {
            dir *= -1; // Make dir point towards A.
        }

        auto projA = -(dot(posA, -dir) + shA.half_length);
        auto projB = shB.support_projection(posB, ornB, dir);
        auto dist = projA - projB;

        if (dist > distance) {
            distance = dist;
            sep_axis = dir;
        }
    }

    // B's faces.
    {
        vector3 dir = axisB;

        if (dot(posA - posB, dir) < 0) {
            dir *= -1; // Make dir point towards A.
        }

        auto projA = -shA.support_projection(posA, ornA, -dir);
        auto projB = dot(posB, dir) + shB.half_length;
        auto dist = projA - projB;

        if (dist > distance) {
            distance = dist;
            sep_axis = dir;
        }
    }

    // Axis vs axis.
    {
        auto dir = cross(axisA, axisB);

        if (try_normalize(dir)) {
            if (dot(posA - posB, dir) < 0) {
                dir *= -1; // Make dir point towards A.
            }

            auto projA = -(dot(posA, -dir) + shA.radius);
            auto projB = dot(posB, dir) + shB.radius;
            auto dist = projA - projB;

            if (dist > distance) {
                distance = dist;
                sep_axis = dir;
            }
        }
    }

    // A's face edges vs B's side edge.
    for (size_t i = 0; i < 2; ++i) {
        for (size_t j = 0; j < 2; ++j) {
            auto is_circleA = j == 0;
            auto circle_pos = is_circleA ? verticesA[i] : verticesB[i];

            // Find closest point between circle and and cylinder axis.
            size_t num_points;
            scalar s0, s1;
            vector3 closest_circle[2];
            vector3 closest_line[2];
            vector3 dir;
            auto orn = is_circleA ? ornA : ornB;
            auto radius = is_circleA ? shA.radius : shB.radius;
            auto &vertices = is_circleA ? verticesB : verticesA;

            closest_point_circle_line(circle_pos, orn, radius,
                                      vertices[0], vertices[1], num_points,
                                      s0, closest_circle[0], closest_line[0],
                                      s1, closest_circle[1], closest_line[1],
                                      dir, support_feature_tolerance);

            // If there are two closest points, it means the segment is parallel
            // to the plane of the circle, which means the separating axis would
            // be a cylinder cap face which was already handled.
            if (num_points == 2) {
                continue;
            }

            if (dot(posA - posB, dir) < 0) {
                dir *= -1; // Make it point towards A.
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

    // Face edges vs face edges.
    for (size_t i = 0; i < 2; ++i) {
        for (size_t j = 0; j < 2; ++j) {
            size_t num_points;
            vector3 closestA[2];
            vector3 closestB[2];
            vector3 dir;
            closest_point_circle_circle(verticesA[i], ornA, shA.radius,
                                        verticesB[j], ornB, shB.radius,
                                        num_points, closestA[0], closestB[0],
                                        closestA[1], closestB[1], dir);
            EDYN_ASSERT(length_sqr(dir) > EDYN_EPSILON);

            if (dot(posA - posB, dir) < 0) {
                dir *= -1; // Make it point towards A.
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

    if (distance > ctx.threshold) {
        return;
    }

    cylinder_feature featureA;
    size_t feature_indexA;
    shA.support_feature(posA, ornA, -sep_axis, featureA, feature_indexA,
                        support_feature_tolerance);

    cylinder_feature featureB;
    size_t feature_indexB;
    shB.support_feature(posB, ornB, sep_axis, featureB, feature_indexB,
                        support_feature_tolerance);

    collision_result::collision_point point;
    point.normal = sep_axis;
    point.distance = distance;
    point.featureA = {featureA, feature_indexA};
    point.featureB = {featureB, feature_indexB};

    auto get_local_distance = [&] (vector3 pivotA, vector3 pivotB) {
        auto pivotA_world = to_world_space(pivotA, posA, ornA);
        auto pivotB_world = to_world_space(pivotB, posB, ornB);
        return dot(pivotA_world - pivotB_world, sep_axis);
    };

    if (featureA == cylinder_feature::face && featureB == cylinder_feature::face) {
        auto posA_in_B = to_object_space(posA, posB, ornB);
        auto ornA_in_B = conjugate(ornB) * ornA;
        point.normal_attachment = contact_normal_attachment::normal_on_B;

        // Intersect the cylinder cap face circles in 2D, in B's space. The
        // cylinder axis is the x axis locally, thus use the z axis in 3D as
        // the x axis in 2D and y axis in 3D as the y axis in 2D.
        vector2 intersection[2];
        auto centerA = to_vector2_zy(posA_in_B);
        auto num_points = intersect_circle_circle(centerA, shA.radius,
                                                  vector2_zero, shB.radius,
                                                  intersection[0], intersection[1]);

        if (num_points > 0) {
            auto merge_distance = contact_breaking_threshold;

            // Merge points if there are two intersections but they're too
            // close to one another.
            if (num_points > 1 && distance_sqr(intersection[0], intersection[1]) < merge_distance * merge_distance) {
                num_points = 1;
                intersection[0] = (intersection[0] + intersection[1]) * scalar(0.5);
            }

            auto pivotA_x = shA.half_length * to_sign(feature_indexA == 0);
            auto pivotB_x = shB.half_length * to_sign(feature_indexB == 0);

            for (size_t i = 0; i < num_points; ++i) {
                point.pivotB = vector3{pivotB_x, intersection[i].y, intersection[i].x};
                point.pivotA = to_object_space(point.pivotB, posA_in_B, ornA_in_B);
                point.pivotA.x = pivotA_x;
                point.distance = get_local_distance(point.pivotA, point.pivotB);
                result.add_point(point);
            }

            auto dist_sqr = length_sqr(centerA);

            // Add extra points to cover the contact area.
            if (num_points > 1) {
                // Circles intersect at two points. Add two extra points in the direction
                // orthogonal to `p[1] - p[0]`. The distance between these points is non-zero
                // here because otherwise they'd have been merged above.
                auto dir = normalize(orthogonal(intersection[1] - intersection[0]));

                // Point in the correct direction, from B to A.
                if (dot(dir, centerA) < 0) {
                    dir *= -1;
                }

                {
                    auto extra_A = centerA - dir * shA.radius;
                    point.pivotB = vector3{pivotB_x, extra_A.y, extra_A.x};
                    point.pivotA = to_object_space(point.pivotB, posA_in_B, ornA_in_B);
                    point.pivotA.x = pivotA_x;
                    // Faces do not line up perfectly, thus calculate the distance
                    // for each pivot.
                    point.distance = get_local_distance(point.pivotA, point.pivotB);
                    result.add_point(point);
                }

                {
                    auto extra_B = dir * shB.radius;
                    point.pivotB = vector3{pivotB_x, extra_B.y, extra_B.x};
                    point.pivotA = to_object_space(point.pivotB, posA_in_B, ornA_in_B);
                    point.pivotA.x = pivotA_x;
                    point.distance = get_local_distance(point.pivotA, point.pivotB);
                    result.add_point(point);
                }
            } else if (dist_sqr < shB.radius * shB.radius || dist_sqr < shA.radius * shA.radius) {
                // Circles intersect at a single point and the center of one is contained
                // within the other. Add 3 extra points on the perimeter of the smaller
                // circle. Guarantted to not be concentric at this point, i.e. `centerA`
                // is not zero.
                auto dir = normalize(centerA);

                // Add one point on the other side of the circle with smaller radius,
                // which in this case is contained within the circle with bigger radius.
                if (shA.radius < shB.radius) {
                    auto extra_A = centerA - dir * shA.radius;
                    point.pivotB = vector3{pivotB_x, extra_A.y, extra_A.x};
                    point.pivotA = to_object_space(point.pivotB, posA_in_B, ornA_in_B);
                    point.pivotA.x = pivotA_x;
                    point.distance = get_local_distance(point.pivotA, point.pivotB);
                    result.add_point(point);
                } else {
                    auto extra_B = dir * shB.radius;
                    point.pivotB = vector3{pivotB_x, extra_B.y, extra_B.x};
                    point.pivotA = to_object_space(point.pivotB, posA_in_B, ornA_in_B);
                    point.pivotA.x = pivotA_x;
                    point.distance = get_local_distance(point.pivotA, point.pivotB);
                    result.add_point(point);
                }

                // Add 2 points in the orthogonal direction.
                dir = orthogonal(dir);

                if (shA.radius < shB.radius) {
                    auto extra_A0 = centerA + dir * shA.radius;
                    point.pivotB = vector3{pivotB_x, extra_A0.y, extra_A0.x};
                    point.pivotA = to_object_space(point.pivotB, posA_in_B, ornA_in_B);
                    point.pivotA.x = pivotA_x;
                    point.distance = get_local_distance(point.pivotA, point.pivotB);
                    result.add_point(point);

                    auto extra_A1 = centerA - dir * shA.radius;
                    point.pivotB = vector3{pivotB_x, extra_A1.y, extra_A1.x};
                    point.pivotA = to_object_space(point.pivotB, posA_in_B, ornA_in_B);
                    point.pivotA.x = pivotA_x;
                    point.distance = get_local_distance(point.pivotA, point.pivotB);
                    result.add_point(point);
                } else {
                    auto extra_B0 = dir * shB.radius;
                    point.pivotB = vector3{pivotB_x, extra_B0.y, extra_B0.x};
                    point.pivotA = to_object_space(point.pivotB, posA_in_B, ornA_in_B);
                    point.pivotA.x = pivotA_x;
                    point.distance = get_local_distance(point.pivotA, point.pivotB);
                    result.add_point(point);

                    auto extra_B1 = -dir * shB.radius;
                    point.pivotB = vector3{pivotB_x, extra_B1.y, extra_B1.x};
                    point.pivotA = to_object_space(point.pivotB, posA_in_B, ornA_in_B);
                    point.pivotA.x = pivotA_x;
                    point.distance = get_local_distance(point.pivotA, point.pivotB);
                    result.add_point(point);
                }
            }
        } else {
            // Check for containment.
            // If any point on a circle of B is within the prism of A, then the face of B
            // is contained in the face of A, and vice-versa.
            auto circle_pointA = posA + quaternion_z(ornA) * shA.radius;
            auto circle_pointB = posB + quaternion_z(ornB) * shB.radius;

            const auto multipliers = std::array<scalar, 4>{0, 1, 0, -1};

            if (distance_sqr_line(posA, axisA, circle_pointB) < shA.radius * shA.radius) {
                auto posB_in_A = to_object_space(posB, posA, ornA);
                auto ornB_in_A = conjugate(ornA) * ornB;

                for(size_t i = 0; i < 4; ++i) {
                    auto pivotB_x = shB.half_length * to_sign(feature_indexB == 0);\
                    point.pivotB = vector3{pivotB_x,
                                           shB.radius * multipliers[i],
                                           shB.radius * multipliers[(i + 1) % 4]};
                    point.pivotA = to_world_space(point.pivotB, posB_in_A, ornB_in_A);
                    point.pivotA.x = shA.half_length * to_sign(feature_indexA == 0);
                    point.distance = get_local_distance(point.pivotA, point.pivotB);
                    result.maybe_add_point(point);
                }
            } else if (distance_sqr_line(posB, axisB, circle_pointA) < shB.radius * shB.radius) {
                for(size_t i = 0; i < 4; ++i) {
                    auto pivotA_x = shA.half_length * to_sign(feature_indexA == 0);
                    point.pivotA = vector3{pivotA_x,
                                           shA.radius * multipliers[i],
                                           shA.radius * multipliers[(i + 1) % 4]};
                    point.pivotB = to_world_space(point.pivotA, posA_in_B, ornA_in_B);
                    point.pivotB.x = shB.half_length * to_sign(feature_indexB == 0);
                    point.distance = get_local_distance(point.pivotA, point.pivotB);
                    result.maybe_add_point(point);
                }
            }
        }
    } else if (featureA == cylinder_feature::face &&
               featureB == cylinder_feature::cap_edge) {
        auto supportB = shB.support_point(posB, ornB, sep_axis);
        auto pivotA_world = project_plane(supportB, verticesA[feature_indexA], sep_axis);
        point.pivotA = to_object_space(pivotA_world, posA, ornA);
        point.pivotB = to_object_space(supportB, posB, ornB);
        point.normal_attachment = contact_normal_attachment::normal_on_A;
        result.maybe_add_point(point);
    } else if (featureA == cylinder_feature::cap_edge &&
               featureB == cylinder_feature::face) {
        auto supportA = shA.support_point(posA, ornA, -sep_axis);
        point.pivotA = to_object_space(supportA, posA, ornA);
        auto pivotB_world = project_plane(supportA, verticesB[feature_indexB], sep_axis);
        point.pivotB = to_object_space(pivotB_world, posB, ornB);
        point.normal_attachment = contact_normal_attachment::normal_on_B;
        result.maybe_add_point(point);
    } else if (featureA == cylinder_feature::face &&
               featureB == cylinder_feature::side_edge) {
        // Attach normal to face of A.
        point.normal_attachment = contact_normal_attachment::normal_on_A;

        // Transform vertices to cylinder space.
        auto v0 = to_object_space(verticesB[0], posA, ornA);
        auto v1 = to_object_space(verticesB[1], posA, ornA);

        scalar s[2];
        auto num_points = intersect_line_circle(to_vector2_zy(v0),
                                                to_vector2_zy(v1),
                                                shA.radius, s[0], s[1]);

        for (size_t i = 0; i < num_points; ++i) {
            s[i] = clamp_unit(s[i]);
            point.pivotA = lerp(v0, v1, s[i]);
            point.pivotA.x = shA.half_length * to_sign(feature_indexA == 0);
            auto normalB = rotate(conjugate(ornB), sep_axis);
            point.pivotB = vector3_x * shB.half_length * (1 - 2 * s[i]) + normalB * shB.radius;
            point.distance = get_local_distance(point.pivotA, point.pivotB);
            result.add_point(point);
        }
    } else if (featureA == cylinder_feature::side_edge &&
               featureB == cylinder_feature::face) {
        // Attach normal to face of B.
        point.normal_attachment = contact_normal_attachment::normal_on_B;

        // Transform vertices to cylinder space.
        auto v0 = to_object_space(verticesA[0], posB, ornB);
        auto v1 = to_object_space(verticesA[1], posB, ornB);

        scalar s[2];
        auto num_points = intersect_line_circle(to_vector2_zy(v0),
                                                to_vector2_zy(v1),
                                                shB.radius, s[0], s[1]);

        for (size_t i = 0; i < num_points; ++i) {
            s[i] = clamp_unit(s[i]);
            point.pivotB = lerp(v0, v1, s[i]);
            point.pivotB.x = shB.half_length * to_sign(feature_indexB == 0);
            auto normalA = rotate(conjugate(ornA), sep_axis);
            point.pivotA = vector3_x * shA.half_length * (1 - 2 * s[i]) - normalA * shA.radius;
            point.distance = get_local_distance(point.pivotA, point.pivotB);
            result.add_point(point);
        }
    } else if (featureA == cylinder_feature::side_edge &&
               featureB == cylinder_feature::side_edge) {
        point.normal_attachment = contact_normal_attachment::none;
        scalar s[2], t[2];
        vector3 closestA[2], closestB[2];
        size_t num_points = 0;
        closest_point_segment_segment(verticesA[0], verticesA[1],
                                      verticesB[0], verticesB[1],
                                      s[0], t[0], closestA[0], closestB[0], &num_points,
                                      &s[1], &t[1], &closestA[1], &closestB[1]);

        for (size_t i = 0; i < num_points; ++i) {
            auto pivotA_world = closestA[i] - sep_axis * shA.radius;
            auto pivotB_world = closestB[i] + sep_axis * shB.radius;
            point.pivotA = to_object_space(pivotA_world, posA, ornA);
            point.pivotB = to_object_space(pivotB_world, posB, ornB);
            result.add_point(point);
        }
    } else if (featureA == cylinder_feature::side_edge &&
               featureB == cylinder_feature::cap_edge) {
        auto supportB = shB.support_point(posB, ornB, sep_axis);
        vector3 pivotA; scalar t;
        closest_point_segment(verticesA[0], verticesA[1], supportB, t, pivotA);

        point.pivotA = to_object_space(pivotA - sep_axis * shA.radius, posA, ornA);
        point.pivotB = to_object_space(supportB, posB, ornB);
        point.normal_attachment = contact_normal_attachment::none;
        result.add_point(point);
    } else if (featureB == cylinder_feature::side_edge &&
               featureA == cylinder_feature::cap_edge) {
        auto supportA = shA.support_point(posA, ornA, -sep_axis);
        vector3 pivotB; scalar t;
        closest_point_segment(verticesB[0], verticesB[1], supportA, t, pivotB);

        point.pivotA = to_object_space(supportA, posA, ornA);
        point.pivotB = to_object_space(pivotB + sep_axis * shB.radius, posB, ornB);
        point.normal_attachment = contact_normal_attachment::none;
        result.add_point(point);
    } else if (featureA == cylinder_feature::cap_edge &&
               featureB == cylinder_feature::cap_edge) {
        auto supportA = shA.support_point(posA, ornA, -sep_axis);
        auto supportB = shB.support_point(posB, ornB, sep_axis);
        point.pivotA = to_object_space(supportA, posA, ornA);
        point.pivotB = to_object_space(supportB, posB, ornB);
        point.normal_attachment = contact_normal_attachment::none;
        result.add_point(point);
    }
}

}
