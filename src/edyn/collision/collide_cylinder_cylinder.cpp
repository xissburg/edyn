#include "edyn/collision/collide.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/math/math.hpp"
#include "edyn/math/scalar.hpp"
#include "edyn/math/vector2.hpp"
#include "edyn/math/vector2_3_util.hpp"
#include "edyn/math/vector3.hpp"
#include <array>

namespace edyn {

struct cyl_cyl_separating_axis {
    vector3 dir;
    cylinder_feature featureA;
    cylinder_feature featureB;
    size_t feature_indexA;
    size_t feature_indexB;
    vector3 pivotA;
    vector3 pivotB;
    scalar distance;
};

collision_result collide(const cylinder_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const cylinder_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold) {
    // Cylinder-cylinder SAT.
    std::array<cyl_cyl_separating_axis, 11> sep_axes;
    size_t axis_idx = 0;

    const auto axisA = quaternion_x(ornA);
    const auto axisB = quaternion_x(ornB);

    const auto half_axisA = axisA * shA.half_length;
    const auto face_center_posA = posA + half_axisA;
    const auto face_center_negA = posA - half_axisA;
    
    const auto half_axisB = axisB * shB.half_length;
    const auto face_center_posB = posB + half_axisB;
    const auto face_center_negB = posB - half_axisB;

    // A's faces.
    {
        auto &axis = sep_axes[axis_idx++];
        axis.featureA = cylinder_feature::face;
        vector3 face_center;

        // Make dir point towards A.
        if (dot(posA - posB, axisA) < 0) {
            axis.dir = -axisA;
            axis.feature_indexA = 0;
            face_center = face_center_posA;
        } else {
            axis.dir = axisA;
            axis.feature_indexA = 1;
            face_center = face_center_negA;
        }

        shB.support_feature(posB, ornB, posA, axis.dir, 
                            axis.featureB, axis.feature_indexB, 
                            axis.pivotB, axis.distance, threshold);
        axis.distance = -(shA.half_length + axis.distance);
        axis.pivotA = project_plane(axis.pivotB, face_center, axis.dir);
    }

    // B's faces.
    {
        auto &axis = sep_axes[axis_idx++];
        axis.featureB = cylinder_feature::face;
        vector3 face_center;

        // Make dir point towards A.
        if (dot(posA - posB, axisB) < 0) {
            axis.dir = -axisB;
            axis.feature_indexB = 1;
            face_center = face_center_negB;
        } else {
            axis.dir = axisB;
            axis.feature_indexB = 0;
            face_center = face_center_posB;
        }

        shA.support_feature(posA, ornA, posB, -axis.dir, 
                            axis.featureA, axis.feature_indexA, 
                            axis.pivotA, axis.distance, threshold);
        axis.distance = -(shB.half_length + axis.distance);
        axis.pivotB = project_plane(axis.pivotA, face_center, axis.dir);
    }

    // Axis vs axis.
    {
        auto dir = cross(axisA, axisB);
        auto dir_len_sqr = length_sqr(dir);

        if (!(dir_len_sqr > EDYN_EPSILON)) {
            // Axes are parallel. Find a vector that's orthogonal to both.
            vector3 closest; scalar t;
            closest_point_line(face_center_negA, axisA, face_center_negB, t, closest);
            dir = closest - face_center_negB;
            dir_len_sqr = length_sqr(dir);
        }

        if (dir_len_sqr > EDYN_EPSILON) {
            dir /= std::sqrt(dir_len_sqr);

            if (dot(posA - posB, dir) < 0) {
                dir *= -1;
            }

            auto &axis = sep_axes[axis_idx++];
            axis.dir = dir;
            
            scalar projA, projB;
            shA.support_feature(posA, ornA, posB, -axis.dir, 
                                axis.featureA, axis.feature_indexA, 
                                axis.pivotA, projA, threshold);
            shB.support_feature(posB, ornB, posB, axis.dir, 
                                axis.featureB, axis.feature_indexB, 
                                axis.pivotB, projB, threshold);
            axis.distance = -(projA + projB);
        }
    }

    // A's Face edges vs B's side edges.
    for (size_t i = 0; i < 2; ++i) {
        const auto is_faceA = i == 0;
        const auto &v0 = is_faceA ? face_center_negB : face_center_negA;
        const auto &v1 = is_faceA ? face_center_posB : face_center_posA;
        const auto &orn = is_faceA ? ornA : ornB;
        const auto radius = is_faceA ? shA.radius : shB.radius;

        for (size_t j = 0; j < 2; ++j) {
            auto circle_pos = is_faceA ? (j == 0 ? face_center_negA : face_center_posA) :
                                         (j == 0 ? face_center_negB : face_center_posB);
            size_t num_points;
            scalar s[2];
            vector3 p_circle[2];
            vector3 p_line[2];
            vector3 normal;
            closest_point_circle_line(circle_pos, orn, radius, v0, v1, num_points, 
                                      s[0], p_circle[0], p_line[0], 
                                      s[1], p_circle[1], p_line[1], normal, threshold);

            auto &axis = sep_axes[axis_idx++];
            axis.dir = normal;
            EDYN_ASSERT(length_sqr(normal) > EDYN_EPSILON);

            if (dot(posA - posB, axis.dir) < 0) {
                // Points towards A.
                axis.dir *= -1;
            }

            if (is_faceA) {
                axis.featureB = cylinder_feature::side_edge;
                shA.support_feature(posA, ornA, posB, -axis.dir, 
                                    axis.featureA, axis.feature_indexA, 
                                    axis.pivotA, axis.distance, threshold);
                axis.distance = -(shB.radius + axis.distance);
            } else {
                axis.featureA = cylinder_feature::side_edge;
                shB.support_feature(posB, ornB, posA, axis.dir, 
                                    axis.featureB, axis.feature_indexB, 
                                    axis.pivotB, axis.distance, threshold);
                axis.distance = -(shA.radius + axis.distance);
            }
        }
    }

    // Face edges vs face edges.
    for (size_t i = 0; i < 2; ++i) {
        auto &circle_posA = i == 0 ? face_center_negA : face_center_posA;

        for (size_t j = 0; j < 2; ++j) {
            auto &circle_posB = i == 0 ? face_center_negB : face_center_posB;
            size_t num_points;
            vector3 closest0[2];
            vector3 closest1[2];
            vector3 normal;
            closest_point_circle_circle(circle_posA, ornA, shA.radius, 
                                        circle_posB, ornB, shB.radius, 
                                        num_points, closest0[0], closest0[1],
                                        closest1[0], closest1[1], normal);

            auto &axis = sep_axes[axis_idx++];
            axis.dir = normal;
            EDYN_ASSERT(length_sqr(normal) > EDYN_EPSILON);

            if (dot(posA - posB, axis.dir) < 0) {
                // Points towards A.
                axis.dir *= -1;
            }

            scalar projA, projB;
            shA.support_feature(posA, ornA, posB, -axis.dir, 
                                axis.featureA, axis.feature_indexA, 
                                axis.pivotA, projA, threshold);
            shB.support_feature(posB, ornB, posB, axis.dir, 
                                axis.featureB, axis.feature_indexB, 
                                axis.pivotB, projB, threshold);
            axis.distance = -(projA + projB);
        }
    }

    auto greatest_distance = -EDYN_SCALAR_MAX;
    size_t sep_axis_idx;

    for (size_t i = 0; i < axis_idx; ++i) {
        auto &sep_axis = sep_axes[i];
        
        if (sep_axis.distance > greatest_distance) {
            greatest_distance = sep_axis.distance;
            sep_axis_idx = i;
        }
    }

    auto &sep_axis = sep_axes[sep_axis_idx];

    if (sep_axis.distance > threshold) {
        return {};
    }

    auto result = collision_result{};
    auto normalB = rotate(conjugate(ornB), sep_axis.dir);

    if (sep_axis.featureA == cylinder_feature::face && 
        sep_axis.featureB == cylinder_feature::face) {

        auto posA_in_B = to_object_space(posA, posB, ornB);
        auto ornA_in_B = conjugate(ornB) * ornA;

        // Intersect the cylinder cap face circles in 2D. The cylinder axis is
        // the x axis locally, thus use the z axis in 3D as the x axis in 2D
        // and y axis in 3D as the y axis in 2D.
        vector2 p[2];
        auto centerA = to_vector2_zy(posA_in_B);
        auto num_points = intersect_circle_circle(centerA, shA.radius, 
                                                  vector2_zero, shB.radius, 
                                                  p[0], p[1]);

        if (num_points > 0) {
            auto pivotA_x = shA.half_length * (sep_axis.feature_indexA == 0 ? 1 : -1);
            auto pivotB_x = shB.half_length * (sep_axis.feature_indexB == 0 ? 1 : -1);
            auto merge_distance = contact_breaking_threshold;

            // Merge points if there are two intersections but they're too
            // close to one another.
            if (num_points > 1 && distance_sqr(p[0], p[1]) < merge_distance * merge_distance) {
                num_points = 1;
                p[0] = (p[0] + p[1]) * 0.5;
            }

            for (size_t i = 0; i < num_points; ++i) {
                auto pivotB = vector3{pivotB_x, p[i].y, p[i].x};
                auto pivotA = to_object_space(pivotB, posA_in_B, ornA_in_B);
                pivotA.x = pivotA_x;
                result.add_point({pivotA, pivotB, normalB, sep_axis.distance});
            }

            auto dist_sqr = length_sqr(centerA);

            // Add extra points to cover the contact area.
            if (num_points > 1) {
                // Circles intersect at two points. Add two extra points in the direction
                // orthogonal `p[1] - p[0]`. The distance between these points is non-zero
                // here because otherwise they'd have been merged above.
                auto dir = normalize(orthogonal(p[1] - p[0]));

                // Point in the correct direction, from B to A.
                if (dot(dir, centerA) < 0) {
                    dir *= -1;
                }

                auto extra_A = centerA - dir * shA.radius;
                auto pivotB = vector3{pivotB_x, extra_A.y, extra_A.x};
                auto pivotA = to_object_space(pivotB, posA_in_B, ornA_in_B);
                pivotA.x = pivotA_x;
                result.add_point({pivotA, pivotB, normalB, sep_axis.distance});

                auto extra_B = dir * shB.radius;
                pivotB = vector3{pivotB_x, extra_B.y, extra_B.x};
                pivotA = to_object_space(pivotB, posA_in_B, ornA_in_B);
                pivotA.x = pivotA_x;
                result.add_point({pivotA, pivotB, normalB, sep_axis.distance});
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
                    auto pivotB = vector3{pivotB_x, extra_A.y, extra_A.x};
                    auto pivotA = to_object_space(pivotB, posA_in_B, ornA_in_B);
                    pivotA.x = pivotA_x;
                    result.add_point({pivotA, pivotB, normalB, sep_axis.distance});
                } else {
                    auto extra_B = dir * shB.radius;
                    auto pivotB = vector3{pivotB_x, extra_B.y, extra_B.x};
                    auto pivotA = to_object_space(pivotB, posA_in_B, ornA_in_B);
                    pivotA.x = pivotA_x;
                    result.add_point({pivotA, pivotB, normalB, sep_axis.distance});
                }

                // Add 2 points in the orthogonal direction.
                dir = orthogonal(dir);

                if (shA.radius < shB.radius) {
                    auto extra_A0 = centerA + dir * shA.radius;
                    auto pivotB = vector3{pivotB_x, extra_A0.y, extra_A0.x};
                    auto pivotA = to_object_space(pivotB, posA_in_B, ornA_in_B);
                    pivotA.x = pivotA_x;
                    result.add_point({pivotA, pivotB, normalB, sep_axis.distance});

                    auto extra_A1 = centerA - dir * shA.radius;
                    pivotB = vector3{pivotB_x, extra_A1.y, extra_A1.x};
                    pivotA = to_object_space(pivotB, posA_in_B, ornA_in_B);
                    pivotA.x = pivotA_x;
                    result.add_point({pivotA, pivotB, normalB, sep_axis.distance});
                } else {
                    auto extra_B0 = dir * shB.radius;
                    auto pivotB = vector3{pivotB_x, extra_B0.y, extra_B0.x};
                    auto pivotA = to_object_space(pivotB, posA_in_B, ornA_in_B);
                    pivotA.x = pivotA_x;
                    result.add_point({pivotA, pivotB, normalB, sep_axis.distance});

                    auto extra_B1 = -dir * shB.radius;
                    pivotB = vector3{pivotB_x, extra_B1.y, extra_B1.x};
                    pivotA = to_object_space(pivotB, posA_in_B, ornA_in_B);
                    pivotA.x = pivotA_x;
                    result.add_point({pivotA, pivotB, normalB, sep_axis.distance});
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
                    auto pivotB_x = shB.half_length * (sep_axis.feature_indexB == 0 ? 1 : -1);
                    auto pivotB = vector3{pivotB_x, 
                                        shB.radius * multipliers[i], 
                                        shB.radius * multipliers[(i + 1) % 4]};
                    auto pivotA = posB_in_A + rotate(ornB_in_A, pivotB);
                    pivotA.x = shA.half_length * (sep_axis.feature_indexA == 0 ? 1 : -1);
                    result.maybe_add_point({pivotA, pivotB, normalB, sep_axis.distance});
                }
            } else if (distance_sqr_line(posB, axisB, circle_pointA) < shB.radius * shB.radius) {
                for(size_t i = 0; i < 4; ++i) {
                    auto pivotA_x = shA.half_length * (sep_axis.feature_indexA == 0 ? 1 : -1);
                    auto pivotA = vector3{pivotA_x, 
                                        shA.radius * multipliers[i], 
                                        shA.radius * multipliers[(i + 1) % 4]};
                    auto pivotB = posA_in_B + rotate(ornA_in_B, pivotA);
                    pivotB.x = shB.half_length * (sep_axis.feature_indexB == 0 ? 1 : -1);
                    result.maybe_add_point({pivotA, pivotB, normalB, sep_axis.distance});
                }
            } 
        }
    } else if ((sep_axis.featureA == cylinder_feature::face && 
                sep_axis.featureB == cylinder_feature::cap_edge) ||
               (sep_axis.featureB == cylinder_feature::face && 
                sep_axis.featureA == cylinder_feature::cap_edge)) {
        auto pivotA = to_object_space(sep_axis.pivotA, posA, ornA);
        auto pivotB = to_object_space(sep_axis.pivotB, posB, ornB);
        result.add_point({pivotA, pivotB, normalB, sep_axis.distance});
    } else if (sep_axis.featureA == cylinder_feature::face &&
               sep_axis.featureB == cylinder_feature::side_edge) {
        // Transform vertices to cylinder space.
        auto v0 = to_object_space(face_center_negB, posA, ornA);
        auto v1 = to_object_space(face_center_posB, posA, ornA);

        scalar s[2];
        auto num_points = intersect_line_circle(to_vector2_zy(v0), 
                                                to_vector2_zy(v1), 
                                                shA.radius, s[0], s[1]);

        for (size_t i = 0; i < num_points; ++i) {
            s[i] = clamp_unit(s[i]);
            auto pivotA = lerp(v0, v1, s[i]);
            pivotA.x = shA.half_length * (sep_axis.feature_indexA == 0 ? 1 : -1);
            auto pivotB = vector3_x * shB.half_length * (2 * s[i] - 1) + normalB * shB.radius;
            result.add_point({pivotA, pivotB, normalB, sep_axis.distance});
        }
    } else if (sep_axis.featureB == cylinder_feature::face &&
               sep_axis.featureA == cylinder_feature::side_edge) {
        // Transform vertices to cylinder space.
        auto v0 = to_object_space(face_center_negA, posB, ornB);
        auto v1 = to_object_space(face_center_posA, posB, ornB);

        scalar s[2];
        auto num_points = intersect_line_circle(to_vector2_zy(v0),
                                                to_vector2_zy(v1), 
                                                shB.radius, s[0], s[1]);

        for (size_t i = 0; i < num_points; ++i) {
            s[i] = clamp_unit(s[i]);
            auto pivotB = lerp(v0, v1, s[i]);
            pivotB.x = shB.half_length * (sep_axis.feature_indexB == 0 ? 1 : -1);
            auto normalA = rotate(conjugate(ornA), sep_axis.dir);
            auto pivotA = vector3_x * shA.half_length * (2 * s[i] - 1) - normalA * shA.radius;
            result.add_point({pivotA, pivotB, normalB, sep_axis.distance});
        }
    } else if (sep_axis.featureA == cylinder_feature::side_edge && 
               sep_axis.featureB == cylinder_feature::side_edge) {
        scalar s[2], t[2];
        vector3 pA[2], pB[2];
        size_t num_points = 0;
        closest_point_segment_segment(face_center_negA, face_center_posA,
                                      face_center_negB, face_center_posB,
                                      s[0], t[0], pA[0], pB[0], &num_points,
                                      &s[1], &t[1], &pA[1], &pB[1]);
        for (size_t i = 0; i < num_points; ++i) {
            auto pivotA_world = pA[i] - sep_axis.dir * shA.radius;
            auto pivotB_world = pB[i] + sep_axis.dir * shB.radius;
            auto pivotA = to_object_space(pivotA_world, posA, ornA);
            auto pivotB = to_object_space(pivotB_world, posB, ornB);
            result.add_point({pivotA, pivotB, normalB, sep_axis.distance});
        }
    } else if (sep_axis.featureA == cylinder_feature::side_edge &&
               sep_axis.featureB == cylinder_feature::cap_edge) {
        vector3 pivotA; scalar t;
        closest_point_segment(face_center_negA, face_center_posA, sep_axis.pivotB, t, pivotA);

        if (t > 0 && t < 1) {
            auto pivotB = to_object_space(sep_axis.pivotB, posB, ornB);
            pivotA = to_object_space(pivotA - sep_axis.dir * shA.radius, posA, ornA);
            result.add_point({pivotA, pivotB, normalB, sep_axis.distance});
        }
    } else if (sep_axis.featureB == cylinder_feature::side_edge &&
               sep_axis.featureA == cylinder_feature::cap_edge) {
        vector3 pivotB; scalar t;
        closest_point_segment(face_center_negB, face_center_posB, sep_axis.pivotA, t, pivotB);

        if (t > 0 && t < 1) {
            auto pivotA = to_object_space(sep_axis.pivotA, posA, ornA);
            pivotB = to_object_space(pivotB + sep_axis.dir * shB.radius, posB, ornB);
            result.add_point({pivotA, pivotB, normalB, sep_axis.distance});
        }
    } else if (sep_axis.featureA == cylinder_feature::cap_edge &&
               sep_axis.featureB == cylinder_feature::cap_edge) {
        auto pivotA = to_object_space(sep_axis.pivotA, posA, ornA);
        auto pivotB = to_object_space(sep_axis.pivotB, posB, ornB);
        result.add_point({pivotA, pivotB, normalB, sep_axis.distance});
    }

    return result;
}

}