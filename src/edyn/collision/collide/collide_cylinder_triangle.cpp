#include "edyn/collision/collide.hpp"
#include "edyn/math/vector2_3_util.hpp"
#include "edyn/math/math.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/shapes/cylinder_shape.hpp"

namespace edyn {

void collide(const cylinder_shape &cylinder, const triangle_shape &tri,
             const collision_context &ctx, collision_result &result) {
    const auto &posA = ctx.posA;
    const auto &ornA = ctx.ornA;

    const auto cylinder_axis = quaternion_x(ornA);
    const vector3 cylinder_vertices[] = {
        posA + cylinder_axis * cylinder.half_length,
        posA - cylinder_axis * cylinder.half_length
    };

    triangle_feature featureB;
    size_t feature_indexB;
    vector3 sep_axis;
    scalar distance = -EDYN_SCALAR_MAX;

    // Cylinder cap normal.
    for (auto i = 0; i < 2; ++i) {
        auto dir = cylinder_axis * to_sign(i);

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
        // `dir` considering it starts at `posA`. The projection of the cylinder
        // onto this axis is minus the half length.
        auto dist = -(cylinder.half_length + proj);

        if (dist > distance) {
            distance = dist;
            sep_axis = dir;
            featureB = tri_feature;
            feature_indexB = tri_feature_idx;
        }
    }

    // Triangle face normal.
    {
        auto projA = -cylinder.support_projection(posA, ornA, -tri.normal);
        auto projB = dot(tri.vertices[0], tri.normal);
        auto dist = projA - projB;

        if (dist > distance) {
            distance = dist;
            sep_axis = tri.normal;
            featureB = triangle_feature::face;
        }
    }

    // Cylinder side edges.
    for (size_t i = 0; i < 3; ++i) {
        auto &axisA = cylinder_axis;
        auto &axisB = tri.edges[i];
        auto dir = cross(axisA, axisB);

        if (!try_normalize(dir)) {
            continue;
        }

        if (dot(posA - tri.vertices[i], dir) < 0) {
            dir *= -1; // Make it point towards cylinder.
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

        auto projA = -cylinder.support_projection(posA, ornA, -dir);
        auto dist = projA - projB;

        if (dist > distance) {
            distance = dist;
            sep_axis = dir;
            featureB = tri_feature;
            feature_indexB = tri_feature_idx;
        }
    }

    // Cylinder cap edges.
    for (size_t i = 0; i < 2; ++i) {
        auto circle_pos = cylinder_vertices[i];

        for (size_t j = 0; j < 3; ++j) {
            auto &v0 = tri.vertices[j];
            auto &v1 = tri.vertices[(j + 1) % 3];

            // Find closest point between circle and triangle edge segment.
            size_t num_points;
            scalar s[2];
            vector3 closest_circle[2];
            vector3 closest_line[2];
            vector3 dir;
            closest_point_circle_line(circle_pos, ornA, cylinder.radius, v0, v1, num_points,
                                      s[0], closest_circle[0], closest_line[0],
                                      s[1], closest_circle[1], closest_line[1],
                                      dir, support_feature_tolerance);

            if (dot(posA - tri.vertices[i], dir) < 0) {
                dir *= -1; // Make it point towards cylinder.
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

            auto projA = -cylinder.support_projection(posA, ornA, -dir);
            auto dist = projA - projB;

            if (dist > distance) {
                distance = dist;
                sep_axis = dir;
                featureB = tri_feature;
                feature_indexB = tri_feature_idx;
            }
        }
    }

    if (distance > ctx.threshold) {
        return;
    }

    cylinder_feature featureA;
    size_t feature_indexA;
    cylinder.support_feature(posA, ornA, -sep_axis, featureA, feature_indexA,
                             support_feature_tolerance);

    if (featureA == cylinder_feature::face && featureB == triangle_feature::face) {
        auto sign_faceA = to_sign(feature_indexA == 0);

        size_t num_edge_intersections = 0;
        size_t last_edge_index = 0;

        // Check if circle and triangle edges intersect.
        for (size_t i = 0; i < 3; ++i) {
            if (tri.ignore_edge(i, sep_axis)) {
                continue;
            }

            // Transform vertices to cylinder space.
            auto v0 = tri.vertices[i];
            auto v0_A = to_object_space(v0, posA, ornA);

            auto v1 = tri.vertices[(i + 1) % 3];
            auto v1_A = to_object_space(v1, posA, ornA);

            scalar s[2];
            auto num_points = intersect_line_circle(to_vector2_zy(v0_A),
                                                    to_vector2_zy(v1_A),
                                                    cylinder.radius, s[0], s[1]);

            if (num_points == 0) {
                continue;
            }

            // Single point outside the segment range.
            if (num_points == 1 && (s[0] < 0 || s[0] > 1)) {
                continue;
            }

            // If both s'es are either below zero or above one, it means the
            // line intersects the circle, but the segment doesn't.
            if (num_points == 2 && ((s[0] < 0 && s[1] < 0) || (s[0] > 1 && s[1] > 1))) {
                continue;
            }

            ++num_edge_intersections;
            last_edge_index = i;

            for (size_t pt_idx = 0; pt_idx < num_points; ++pt_idx) {
                auto t = s[pt_idx];
                // Avoid adding the same point twice.
                if (!(t < 1)) continue;

                auto u = clamp_unit(t);
                auto pivotA_x = cylinder.half_length * sign_faceA;
                auto pivotA = lerp(v0_A, v1_A, u);
                auto local_distance = (pivotA.x - pivotA_x) * sign_faceA;
                pivotA.x = pivotA_x;
                auto pivotB = lerp(v0, v1, u);
                result.maybe_add_point({pivotA, pivotB, sep_axis, local_distance});
            }
        }

        // If there are no edge intersections, it means the circle could be
        // contained in the triangle.
        if (num_edge_intersections == 0) {
            // Check if cylinder center is contained in the triangle prism.
            if (point_in_triangle(tri.vertices, tri.normal, posA)) {
                auto multipliers = std::array<scalar, 4>{0, 1, 0, -1};
                for(size_t i = 0; i < 4; ++i) {
                    auto pivotA_x = cylinder.half_length * sign_faceA;
                    auto pivotA = vector3{pivotA_x,
                                          cylinder.radius * multipliers[i],
                                          cylinder.radius * multipliers[(i + 1) % 4]};
                    auto pivotB = to_world_space(pivotA, posA, ornA);
                    pivotB = project_plane(pivotB, tri.vertices[0], tri.normal);
                    result.maybe_add_point({pivotA, pivotB, sep_axis, distance});
                }
            }
        } else if (num_edge_intersections == 1) {
            // If it intersects a single edge, only two contact points have been added,
            // thus add extra points to create a stable base.
            auto tangent = cross(tri.normal, tri.edges[last_edge_index]);
            auto other_vertex_idx = (last_edge_index + 2) % 3;
            if (dot(tangent, tri.vertices[other_vertex_idx] - tri.vertices[last_edge_index]) < 0) {
                tangent *= -1;
            }

            tangent = normalize(tangent);
            auto circle_pos = cylinder_vertices[feature_indexA];
            auto pivotA_in_B = circle_pos + tangent * cylinder.radius;
            auto pivotA = to_object_space(pivotA_in_B, posA, ornA);
            auto pivotB = project_plane(pivotA_in_B, tri.vertices[0], tri.normal);
            result.maybe_add_point({pivotA, pivotB, sep_axis, distance});
        }
    } else if (featureA == cylinder_feature::face && featureB == triangle_feature::edge) {
        // Transform vertices to cylinder space.
        auto v0 = tri.vertices[feature_indexB];
        auto v0_A = to_object_space(v0, posA, ornA);

        auto v1 = tri.vertices[(feature_indexB + 1) % 3];
        auto v1_A = to_object_space(v1, posA, ornA);

        scalar s[2];
        auto num_points = intersect_line_circle(to_vector2_zy(v0_A),
                                                to_vector2_zy(v1_A),
                                                cylinder.radius, s[0], s[1]);
        auto sign_faceA = to_sign(feature_indexA == 0);

        for (size_t pt_idx = 0; pt_idx < num_points; ++pt_idx) {
            auto t = clamp_unit(s[pt_idx]);
            auto pivotA_x = cylinder.half_length * sign_faceA;
            auto pivotA = lerp(v0_A, v1_A, t);
            auto local_distance = (pivotA.x - pivotA_x) * sign_faceA;
            pivotA.x = pivotA_x;
            auto pivotB = lerp(v0, v1, t);
            result.maybe_add_point({pivotA, pivotB, sep_axis, local_distance});
        }
    } else if (featureA == cylinder_feature::face && featureB == triangle_feature::vertex) {
        auto sign_faceA = to_sign(feature_indexA == 0);
        auto vertexB = tri.vertices[feature_indexB];
        auto vertexA_x = cylinder.half_length * sign_faceA;
        auto vertexA = to_object_space(vertexB, posA, ornA);
        auto local_distance = (vertexA.x - vertexA_x) * sign_faceA;
        vertexA.x = vertexA_x; // Project onto face by setting the x value directly.
        result.maybe_add_point({vertexA, vertexB, sep_axis, local_distance});
    } else if (featureA == cylinder_feature::side_edge && featureB == triangle_feature::face) {
        // Cylinder is on its side laying on the triangle face.
        // Check if cylinder vertices are inside triangle face.
        for (auto &vertex : cylinder_vertices) {
            if (point_in_triangle(tri.vertices, tri.normal, vertex)) {
                auto pivotA_world = vertex - tri.normal * cylinder.radius;
                auto pivotA = to_object_space(pivotA_world, posA, ornA);
                auto pivotB = project_plane(vertex, tri.vertices[0], tri.normal);
                auto local_distance = dot(pivotA_world - tri.vertices[0], tri.normal);
                result.maybe_add_point({pivotA, pivotB, sep_axis, local_distance});
            }
        }

        // Both vertices are inside the triangle. Unnecessary to look for intersections.
        if (result.num_points == 2) {
            return;
        }

        // Check if the cylinder edge intersects the triangle edges.
        auto &tri_origin = tri.vertices[0];
        auto tangent = normalize(tri.vertices[1] - tri.vertices[0]);
        auto bitangent = cross(tri.normal, tangent);
        auto tri_basis = matrix3x3_columns(tangent, tri.normal, bitangent);

        auto p0 = to_vector2_xz(to_object_space(cylinder_vertices[0], tri_origin, tri_basis));
        auto p1 = to_vector2_xz(to_object_space(cylinder_vertices[1], tri_origin, tri_basis));

        for (int i = 0; i < 3; ++i) {
            // Ignore concave edges.
            if (tri.is_concave_edge[i]) {
                continue;
            }

            auto &v0 = tri.vertices[i];
            auto &v1 = tri.vertices[(i + 1) % 3];
            auto q0 = to_vector2_xz(to_object_space(v0, tri_origin, tri_basis));
            auto q1 = to_vector2_xz(to_object_space(v1, tri_origin, tri_basis));

            scalar s[2], t[2];
            auto num_points = intersect_segments(p0, p1, q0, q1, s[0], t[0], s[1], t[1]);

            for (size_t k = 0; k < num_points; ++k) {
                auto pivotA_world = lerp(cylinder_vertices[0], cylinder_vertices[1], s[k]) - sep_axis * cylinder.radius;
                auto pivotA = to_object_space(pivotA_world, posA, ornA);
                auto pivotB = lerp(v0, v1, t[k]);
                auto local_distance = dot(pivotA_world - tri.vertices[0], sep_axis);
                result.maybe_add_point({pivotA, pivotB, sep_axis, local_distance});
            }
        }
    } else if (featureA == cylinder_feature::side_edge && featureB == triangle_feature::edge) {
        auto v0 = tri.vertices[feature_indexB];
        auto v1 = tri.vertices[(feature_indexB + 1) % 3];
        scalar s[2], t[2];
        vector3 p0[2], p1[2];
        size_t num_points = 0;
        closest_point_segment_segment(cylinder_vertices[1], cylinder_vertices[0], v0, v1,
                                        s[0], t[0], p0[0], p1[0], &num_points,
                                        &s[1], &t[1], &p0[1], &p1[1]);

        for (size_t i = 0; i < num_points; ++i) {
            auto pA_in_B = p0[i] - sep_axis * cylinder.radius;
            auto pA = to_object_space(pA_in_B, posA, ornA);
            result.maybe_add_point({pA, p1[i], sep_axis, distance});
        }
    } else if (featureA == cylinder_feature::side_edge && featureB == triangle_feature::vertex) {
        auto pivotB = tri.vertices[feature_indexB];
        vector3 pivotA; scalar t;
        closest_point_segment(cylinder_vertices[0], cylinder_vertices[1], pivotB, t, pivotA);
        pivotA -= sep_axis * cylinder.radius;
        pivotA = to_object_space(pivotA, posA, ornA);
        result.maybe_add_point({pivotA, pivotB, sep_axis, distance});
    } else if (featureA == cylinder_feature::cap_edge) {
        auto pivotA_world = cylinder.support_point(posA, ornA, -sep_axis);

        // Do not add point if it's a triangle face and the point lies outside.
        if (featureB != triangle_feature::face ||
            point_in_triangle(tri.vertices, tri.normal, pivotA_world)) {
            auto pivotA = to_object_space(pivotA_world, posA, ornA);
            auto pivotB = pivotA_world - sep_axis * distance;
            result.maybe_add_point({pivotA, pivotB, sep_axis, distance});
        }
    }
}

}
