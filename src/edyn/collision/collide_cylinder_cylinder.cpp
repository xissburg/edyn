#include "edyn/collision/collide.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/math/math.hpp"
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

            if (dot(posA - posB, axis.dir) < 0) {
                // Points towards A.
                axis.dir *= -1;
            }

            if (is_faceA) {
                axis.featureB = cylinder_feature::edge;
                shA.support_feature(posA, ornA, posB, -axis.dir, 
                                    axis.featureA, axis.feature_indexA, 
                                    axis.pivotA, axis.distance, threshold);
                axis.distance = -(shB.radius + axis.distance);
            } else {
                axis.featureA = cylinder_feature::edge;
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

        vector3 p[2];
        auto num_points = intersect_circle_circle(posA_in_B.z, posA_in_B.y, 0, 0, 
                                                  shA.radius, shB.radius, 
                                                  p[0].z, p[0].y, p[1].z, p[1].y);
        if (num_points > 0) {
            for (size_t i = 0; i < num_points; ++i) {
                auto pivotB = p[i];
                pivotB.x = shB.half_length * (sep_axis.feature_indexB == 0 ? 1 : -1);
                auto pivotA = to_object_space(pivotB, posA_in_B, ornA_in_B);
                pivotA.x = shA.half_length * (sep_axis.feature_indexA == 0 ? 1 : -1);
                result.add_point({pivotA, pivotB, normalB, sep_axis.distance});
            }
        // Check for containment.
        } else {
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
               sep_axis.featureB == cylinder_feature::edge) {
        // Transform vertices to cylinder space.
        auto v0 = to_object_space(face_center_negB, posA, ornA);
        auto v1 = to_object_space(face_center_posB, posA, ornA);

        scalar s[2];
        auto num_points = intersect_line_circle(v0.z, v0.y, v1.z, v1.y, 
                                                shA.radius, s[0], s[1]);

        for (size_t i = 0; i < num_points; ++i) {
            s[i] = clamp_unit(s[i]);
            auto pivotA = lerp(v0, v1, s[i]);
            pivotA.x = shA.half_length * (sep_axis.feature_indexA == 0 ? 1 : -1);
            auto pivotB = vector3_x * shB.half_length * (2 * s[i] - 1) + normalB * shB.radius;
            result.add_point({pivotA, pivotB, normalB, sep_axis.distance});
        }
    } else if (sep_axis.featureB == cylinder_feature::face &&
               sep_axis.featureA == cylinder_feature::edge) {
        // Transform vertices to cylinder space.
        auto v0 = to_object_space(face_center_negA, posB, ornB);
        auto v1 = to_object_space(face_center_posA, posB, ornB);

        scalar s[2];
        auto num_points = intersect_line_circle(v0.z, v0.y, v1.z, v1.y, 
                                                shB.radius, s[0], s[1]);

        for (size_t i = 0; i < num_points; ++i) {
            s[i] = clamp_unit(s[i]);
            auto pivotB = lerp(v0, v1, s[i]);
            pivotB.x = shB.half_length * (sep_axis.feature_indexB == 0 ? 1 : -1);
            auto normalA = rotate(conjugate(ornA), sep_axis.dir);
            auto pivotA = vector3_x * shA.half_length * (2 * s[i] - 1) - normalA * shA.radius;
            result.add_point({pivotA, pivotB, normalB, sep_axis.distance});
        }
    } else if (sep_axis.featureA == cylinder_feature::edge && 
               sep_axis.featureB == cylinder_feature::edge) {
        scalar s[2], t[2];
        vector3 pA[2], pB[2];
        size_t num_points = 0;
        closest_point_segment_segment(face_center_negA, face_center_posA,
                                      face_center_negB, face_center_posB,
                                      s[0], t[0], pA[0], pB[0], &num_points,
                                      &s[1], &t[1], &pA[1], &pB[1]);
        for (size_t i = 0; i < num_points; ++i) {
            if (s[i] > 0 && s[i] < 1 && t[i] > 0 && t[i] < 1) {
                auto pivotA_world = pA[i] - sep_axis.dir * shA.radius;
                auto pivotB_world = pB[i] + sep_axis.dir * shB.radius;
                auto pivotA = to_object_space(pivotA_world, posA, ornA);
                auto pivotB = to_object_space(pivotB_world, posB, ornB);
                result.add_point({pivotA, pivotB, normalB, sep_axis.distance});
            }
        }
    } else if (sep_axis.featureA == cylinder_feature::edge &&
               sep_axis.featureB == cylinder_feature::cap_edge) {
        vector3 pivotA; scalar t;
        closest_point_segment(face_center_negA, face_center_posA, sep_axis.pivotB, t, pivotA);

        if (t > 0 && t < 1) {
            auto pivotB = to_object_space(sep_axis.pivotB, posB, ornB);
            pivotA = to_object_space(pivotA - sep_axis.dir * shA.radius, posA, ornA);
            result.add_point({pivotA, pivotB, normalB, sep_axis.distance});
        }
    } else if (sep_axis.featureB == cylinder_feature::edge &&
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