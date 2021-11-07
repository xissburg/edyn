#include "edyn/collision/collide.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/math/vector2_3_util.hpp"
#include "edyn/math/math.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/math/transform.hpp"
#include "edyn/shapes/cylinder_shape.hpp"
#include "edyn/util/shape_util.hpp"
#include "edyn/util/triangle_util.hpp"

namespace edyn {

void collide_cylinder_triangle(
    const cylinder_shape &cylinder, const triangle_mesh &mesh, size_t tri_idx,
    const vector3 &cylinder_axis, const std::array<vector3, 2> &cylinder_vertices,
    const collision_context &ctx, collision_result &result) {

    const auto &posA = ctx.posA;
    const auto &ornA = ctx.ornA;
    const auto tri_vertices = mesh.get_triangle_vertices(tri_idx);
    const auto tri_normal = mesh.get_triangle_normal(tri_idx);
    const auto tri_center = average(tri_vertices);

    auto distance = -EDYN_SCALAR_MAX;
    auto sep_axis = vector3_zero;

    // Check if the given direction is the best so far and set it if so.
    auto test_direction = [&] (vector3 dir) {
        auto projA = -cylinder.support_projection(posA, ornA, -dir);
        auto projB = get_triangle_support_projection(tri_vertices, dir);
        auto dist = projA - projB;

        if (dist > distance) {
            distance = dist;
            sep_axis = dir;
        }
    };

    // Triangle face normal.
    {
        auto projA = -cylinder.support_projection(posA, ornA, -tri_normal);
        auto projB = dot(tri_vertices[0], tri_normal);
        distance = projA - projB;
        sep_axis = tri_normal;
    }

    // Cylinder cap normals.
    {
        auto dir = cylinder_axis;

        if (dot(posA - tri_center, dir) < 0) {
            dir *= -1; // Make it point towards cylinder.
        }

        auto projA = -(dot(posA, -dir) + cylinder.half_length);
        auto projB = get_triangle_support_projection(tri_vertices, dir);
        auto dist = projA - projB;

        if (dist > distance) {
            distance = dist;
            sep_axis = dir;
        }
    }

    // Cylinder side edge vs Triangle edges.
    for (size_t i = 0; i < 3; ++i) {
        auto j = (i + 1) % 3;
        auto v0 = tri_vertices[i];
        auto v1 = tri_vertices[j];
        auto edge_dir = v1 - v0;
        auto dir = cross(edge_dir, cylinder_axis);

        if (!try_normalize(dir)) {
            continue;
        }

        if (dot(posA - tri_center, dir) < 0) {
            dir *= -1; // Make it point towards cylinder.
        }

        test_direction(dir);
    }

    // Cylinder cap face edges.
    for (size_t i = 0; i < 2; ++i) {
        auto circle_pos = cylinder_vertices[i];

        for (size_t j = 0; j < 3; ++j) {
            auto v0 = tri_vertices[j];
            auto v1 = tri_vertices[(j + 1) % 3];

            // Find closest point between circle and triangle edge line.
            size_t num_points;
            scalar s[2];
            vector3 closest_circle[2], closest_line[2];
            vector3 dir;
            closest_point_circle_line(circle_pos, ornA, cylinder.radius, v0, v1, num_points,
                                        s[0], closest_circle[0], closest_line[0],
                                        s[1], closest_circle[1], closest_line[1],
                                        dir, support_feature_tolerance);

            if (dot(posA - tri_center, dir) < 0) {
                dir *= -1; // Make it point towards cylinder.
            }

            test_direction(dir);
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

    sep_axis = clip_triangle_separating_axis(sep_axis, mesh, tri_idx, tri_vertices, tri_normal, tri_feature, tri_feature_index);

    get_triangle_support_feature(tri_vertices, vector3_zero, sep_axis,
                                 tri_feature, tri_feature_index,
                                 proj_tri, support_feature_tolerance);

    auto proj_cyl = -cylinder.support_projection(posA, ornA, -sep_axis);

    distance = proj_cyl - proj_tri;

    if (distance > ctx.threshold) {
        return;
    }

    cylinder_feature cyl_feature;
    size_t cyl_feature_index;
    cylinder.support_feature(posA, ornA, -sep_axis, cyl_feature, cyl_feature_index,
                             support_feature_tolerance);

    collision_result::collision_point point;
    point.normal = sep_axis;
    point.distance = distance;
    point.featureA = {cyl_feature, cyl_feature_index};
    point.featureB = {tri_feature};
    point.featureB->index = get_triangle_mesh_feature_index(mesh, tri_idx, tri_feature, tri_feature_index);

    if (cyl_feature == cylinder_feature::face && tri_feature == triangle_feature::face) {
        size_t num_vertices_in_face = 0;
        auto sign_faceA = to_sign(cyl_feature_index == 0);
        point.normal_attachment = contact_normal_attachment::normal_on_B;

        // Check if triangle vertices are inside a cylinder cap face (by checking
        // if its distance from the cylinder axis is smaller than the cylinder radius).
        for (auto vertex : tri_vertices) {
            vector3 closest; scalar t;
            auto dist_sqr = closest_point_line(posA, cylinder_axis, vertex, t, closest);

            if (dist_sqr > cylinder.radius * cylinder.radius) continue;

            point.pivotA = to_object_space(vertex, posA, ornA);
            point.pivotA.x = cylinder.half_length * sign_faceA;
            point.pivotB = vertex;
            result.maybe_add_point(point);

            ++num_vertices_in_face;
        }

        // If all triangle vertices are contained in the cap face, there's
        // nothing else to be done.
        if (num_vertices_in_face == 3) {
            return;
        }

        // Add points for the cylinder cap edge perimeter inside the triangle.
        auto multipliers = std::array<scalar, 4>{0, 1, 0, -1};
        for (auto i = 0; i < 4; ++i) {
            auto j = (i + 1) % 4;
            auto pivotA_x = cylinder.half_length * sign_faceA;
            point.pivotA = vector3{pivotA_x,
                                   cylinder.radius * multipliers[i],
                                   cylinder.radius * multipliers[j]};
            auto pivotA_world = to_world_space(point.pivotA, posA, ornA);

            if (!point_in_triangle(tri_vertices, tri_normal, pivotA_world)) {
                continue;
            }

            point.distance = dot(pivotA_world - tri_vertices[0], tri_normal);
            point.pivotB = pivotA_world - tri_normal * point.distance;
            result.maybe_add_point(point);
        }

        // Check if circle and triangle edges intersect.
        for (int i = 0; i < 3; ++i) {
            // Ignore concave edges.
            if (!mesh.is_convex_edge(mesh.get_face_edge_index(tri_idx, i))) {
                continue;
            }

            // Transform vertices to cylinder space.
            auto v0 = tri_vertices[i];
            auto v0_A = to_object_space(v0, posA, ornA);

            auto v1 = tri_vertices[(i + 1) % 3];
            auto v1_A = to_object_space(v1, posA, ornA);

            scalar s[2];
            auto num_points = intersect_line_circle(to_vector2_zy(v0_A),
                                                    to_vector2_zy(v1_A),
                                                    cylinder.radius, s[0], s[1]);

            for (auto j = num_points; j > 0; --j) {
                auto t = s[j - 1];

                if (!(t > 0 && t < 1)) continue;

                auto pivotA_x = cylinder.half_length * sign_faceA;
                point.pivotA = lerp(v0_A, v1_A, t);
                point.pivotA.x = pivotA_x;
                point.pivotB = lerp(v0, v1, t);
                result.maybe_add_point(point);
            }
        }
    } else if (cyl_feature == cylinder_feature::face && tri_feature == triangle_feature::edge) {
        EDYN_ASSERT(mesh.is_convex_edge(mesh.get_face_edge_index(tri_idx, tri_feature_index)));

        vector3 edge_vertices[] = {tri_vertices[tri_feature_index],
                                   tri_vertices[(tri_feature_index + 1) % 3]};

        // Check if circle and edge intersect.
        // Transform vertices to cylinder space. The cylinder axis is the x-axis.
        auto v0A = to_object_space(edge_vertices[0], posA, ornA);
        auto v1A = to_object_space(edge_vertices[1], posA, ornA);

        scalar s[2];
        auto v0A_zy = to_vector2_zy(v0A);
        auto v1A_zy = to_vector2_zy(v1A);
        auto num_points = intersect_line_circle(v0A_zy, v1A_zy,
                                                cylinder.radius, s[0], s[1]);

        auto sign_faceA = to_sign(cyl_feature_index == 0);
        auto pivotA_x = cylinder.half_length * sign_faceA;
        point.normal_attachment = contact_normal_attachment::normal_on_A;

        for (size_t pt_idx = 0; pt_idx < num_points; ++pt_idx) {
            auto t = clamp_unit(s[pt_idx]);
            point.pivotA = lerp(v0A, v1A, t);
            point.distance = (point.pivotA.x - pivotA_x) * sign_faceA;
            point.pivotA.x = pivotA_x;
            point.pivotB = lerp(edge_vertices[0], edge_vertices[1], t);
            result.maybe_add_point(point);
        }
    } else if (cyl_feature == cylinder_feature::face && tri_feature == triangle_feature::vertex) {
        auto vertex = tri_vertices[tri_feature_index];
        vector3 closest; scalar t;
        auto dist_sqr = closest_point_line(posA, cylinder_axis, vertex, t, closest);

        if (dist_sqr < cylinder.radius * cylinder.radius) {
            point.pivotA = to_object_space(vertex, posA, ornA);
            point.pivotA.x = cylinder.half_length * to_sign(cyl_feature_index == 0);
            point.pivotB = vertex;
            point.normal_attachment = contact_normal_attachment::normal_on_A;
            result.maybe_add_point(point);
        }
    } else if (cyl_feature == cylinder_feature::side_edge && tri_feature == triangle_feature::face) {
        // Check if edge vertices are inside triangle face.
        size_t num_edge_vert_in_tri_face = 0;

        auto radial_dir = normalize(project_direction(-sep_axis, cylinder_axis));
        point.normal_attachment = contact_normal_attachment::normal_on_B;

        for (auto vertex : cylinder_vertices) {
            if (point_in_triangle(tri_vertices, tri_normal, vertex)) {
                auto pivotA_world = vertex + radial_dir * cylinder.radius;
                point.pivotA = to_object_space(pivotA_world, posA, ornA);
                point.distance = dot(pivotA_world - tri_vertices[0], sep_axis);
                point.pivotB = pivotA_world - sep_axis * point.distance;
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

        auto v0A_in_tri = to_object_space(cylinder_vertices[0], tri_origin, tri_basis);
        auto v1A_in_tri = to_object_space(cylinder_vertices[1], tri_origin, tri_basis);
        auto p0 = to_vector2_xz(v0A_in_tri);
        auto p1 = to_vector2_xz(v1A_in_tri);

        for (int i = 0; i < 3; ++i) {
            // Ignore concave edges.
            if (!mesh.is_convex_edge(mesh.get_face_edge_index(tri_idx, i))) {
                continue;
            }

            auto &v0 = tri_vertices[i];
            auto &v1 = tri_vertices[(i + 1) % 3];

            auto v0B_in_tri = to_object_space(v0, tri_origin, tri_basis);
            auto v1B_in_tri = to_object_space(v1, tri_origin, tri_basis);
            auto q0 = to_vector2_xz(v0B_in_tri);
            auto q1 = to_vector2_xz(v1B_in_tri);

            scalar s[2], t[2];
            auto num_points = intersect_segments(p0, p1, q0, q1, s[0], t[0], s[1], t[1]);

            for (size_t k = 0; k < num_points; ++k) {
                auto pivotA_world = lerp(cylinder_vertices[0], cylinder_vertices[1], s[k]) + radial_dir * cylinder.radius;
                point.pivotA = to_object_space(pivotA_world, posA, ornA);
                point.pivotB = lerp(v0, v1, t[k]);
                point.distance = dot(pivotA_world - point.pivotB, sep_axis);
                result.maybe_add_point(point);
            }
        }
    } else if (cyl_feature == cylinder_feature::side_edge && tri_feature == triangle_feature::edge) {
        auto v0 = tri_vertices[tri_feature_index];
        auto v1 = tri_vertices[(tri_feature_index + 1) % 3];
        scalar s[2], t[2];
        vector3 closestA[2], closestB[2];
        size_t num_points = 0;
        closest_point_segment_segment(cylinder_vertices[1], cylinder_vertices[0], v0, v1,
                                      s[0], t[0], closestA[0], closestB[0], &num_points,
                                      &s[1], &t[1], &closestA[1], &closestB[1]);

        if (num_points > 0) {
            point.normal_attachment = contact_normal_attachment::none;

            // If there's a single closest point, ignore it if it's at a vertex.
            // If there are more than one point, the segments are parallel and
            // both closest points should be considered.
            if ((num_points == 1 && s[0] > 0 && s[0] < 1 && t[0] > 0 && t[0] < 1) || num_points > 1) {
                for (size_t i = 0; i < num_points; ++i) {
                    auto pivotA_world = closestA[i] - sep_axis * cylinder.radius;
                    point.pivotA = to_object_space(pivotA_world, posA, ornA);
                    point.pivotB = closestB[i];
                    result.maybe_add_point(point);
                }
            }
        }
    } else if (cyl_feature == cylinder_feature::side_edge && tri_feature == triangle_feature::vertex) {
        auto vertex = tri_vertices[tri_feature_index];
        vector3 closest; scalar t;
        auto cyl_dir = cylinder_vertices[1] - cylinder_vertices[0];
        closest_point_line(cylinder_vertices[0], cyl_dir, vertex, t, closest);

        // Check if closest point is on the cylinder axis.
        if (t > 0 && t < 1) {
            // Add point to result if the closest point match or the vector
            // connecting them is parallel to the separating axis.
            auto dir = closest - vertex;
            auto dir_len_sqr = length_sqr(dir);

            // They're parallel if the absolute dot product is equals to the product
            // of their lengths.
            if (!(std::abs(square(dot(dir, sep_axis)) - dir_len_sqr) > EDYN_EPSILON)) {
                auto pivotA_world = closest - sep_axis * cylinder.radius;
                point.pivotA = to_object_space(pivotA_world, posA, ornA);
                point.pivotB = vertex;
                point.normal_attachment = contact_normal_attachment::none;
                result.maybe_add_point(point);
            }
        }
    } else if (cyl_feature == cylinder_feature::cap_edge && tri_feature == triangle_feature::face) {
        auto supportA = support_point_circle(cylinder_vertices[cyl_feature_index], ornA, cylinder.radius, -sep_axis);

        if (point_in_triangle(tri_vertices, tri_normal, supportA)) {
            point.pivotA = to_object_space(supportA, posA, ornA);
            point.pivotB = project_plane(supportA, tri_vertices[0], tri_normal);
            point.normal_attachment = contact_normal_attachment::normal_on_B;
            point.normal = tri_normal;
            result.maybe_add_point(point);
        }
    } else if (cyl_feature == cylinder_feature::cap_edge && tri_feature == triangle_feature::edge) {
        auto supportA = support_point_circle(cylinder_vertices[cyl_feature_index], ornA, cylinder.radius, -sep_axis);
        auto v0 = tri_vertices[tri_feature_index];
        auto v1 = tri_vertices[(tri_feature_index + 1) % 3];
        vector3 closest; scalar t;
        closest_point_line(v0, v1 - v0, supportA, t, closest);

        // Check if closest point is on the triangle edge segment.
        if (t > 0 && t < 1) {
            // Add point to result if the closest point match or the vector
            // connecting them is parallel to the separating axis.
            auto dir = supportA - closest;
            auto dir_len_sqr = length_sqr(dir);

            if (!(std::abs(square(dot(dir, sep_axis)) - dir_len_sqr) > EDYN_EPSILON)) {
                point.pivotA = to_object_space(supportA, posA, ornA);
                point.pivotB = closest;
                point.normal_attachment = contact_normal_attachment::none;
                result.maybe_add_point(point);
            }
        }
    }
}

void collide(const cylinder_shape &cylinder, const triangle_mesh &mesh,
             const collision_context &ctx, collision_result &result) {
    const auto cylinder_axis = quaternion_x(ctx.ornA);
    const auto cylinder_vertices = std::array<vector3, 2>{
        ctx.posA + cylinder_axis * cylinder.half_length,
        ctx.posA - cylinder_axis * cylinder.half_length
    };

    const auto inset = vector3_one * -contact_breaking_threshold;
    const auto visit_aabb = ctx.aabbA.inset(inset);

    mesh.visit_triangles(visit_aabb, [&] (auto tri_idx) {
        collide_cylinder_triangle(cylinder, mesh, tri_idx,
                                  cylinder_axis, cylinder_vertices, ctx, result);
    });
}

}
