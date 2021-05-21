#include "edyn/collision/collide.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/math/vector2_3_util.hpp"
#include "edyn/math/math.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/shapes/cylinder_shape.hpp"
#include "edyn/shapes/triangle_shape.hpp"

namespace edyn {

void collide(const cylinder_shape &cylinder, const triangle_mesh &mesh,
             const collision_context &ctx, collision_result &result) {
    const auto &posA = ctx.posA;
    const auto &ornA = ctx.ornA;

    const auto cylinder_axis = quaternion_x(ornA);
    const vector3 cylinder_vertices[] = {
        posA + cylinder_axis * cylinder.half_length,
        posA - cylinder_axis * cylinder.half_length
    };

    const auto inset = vector3_one * -contact_breaking_threshold;
    const auto visit_aabb = ctx.aabbA.inset(inset);

    mesh.visit_vertices(visit_aabb, [&] (auto vertex_idx) {
        if (mesh.is_concave_vertex(vertex_idx)) {
            return;
        }

        auto vertex = mesh.get_vertex_position(vertex_idx);
        auto sep_axis = vector3_zero;
        auto distance = -EDYN_SCALAR_MAX;

        // Check cylinder cap faces.
        auto axis0 = -cylinder_axis;

        if (mesh.in_vertex_voronoi(vertex_idx, axis0)) {
            auto dist0 = dot(cylinder_vertices[0] - vertex, axis0);
            distance = dist0;
            sep_axis = axis0;
        }

        auto axis1 = cylinder_axis;

        if (mesh.in_vertex_voronoi(vertex_idx, axis1)) {
            auto dist1 = dot(cylinder_vertices[1] - vertex, axis1);

            if (dist1 > distance) {
                distance = dist1;
                sep_axis = axis1;
            }
        }

        // Check cylinder axis.
        vector3 closest; scalar t;
        closest_point_line(cylinder_vertices[0], cylinder_vertices[1] - cylinder_vertices[0], vertex, t, closest);
        auto closest_dir = closest - vertex;

        if (try_normalize(closest_dir) && mesh.in_vertex_voronoi(vertex_idx, closest_dir)) {
            auto dist_line = dot(closest - vertex, closest_dir) - cylinder.radius;

            if (dist_line > distance) {
                distance = dist_line;
                sep_axis = closest_dir;
            }
        }

        if (distance > ctx.threshold || distance == -EDYN_SCALAR_MAX) {
            return;
        }

        auto pivotA_world = vertex + sep_axis * distance;
        auto pivotA = to_object_space(pivotA_world, posA, ornA);
        auto pivotB = vertex;
        result.maybe_add_point({pivotA, pivotB, sep_axis, distance});
    });

    mesh.visit_edges(visit_aabb, [&] (auto edge_idx) {
        if (mesh.is_concave_edge(edge_idx)) {
            return;
        }

        auto edge_vertices = mesh.get_edge_vertices(edge_idx);
        auto edge_dir = edge_vertices[1] - edge_vertices[0];
        auto face_normals = mesh.get_convex_edge_face_normals(edge_idx);

        // Do not consider this edge if the neighboring faces have a smaller
        // amount of penetration, i.e. a bigger distance.
        auto distances_faces = std::array<scalar, face_normals.size()>{};

        for (auto i = 0; i < face_normals.size(); ++i) {
            auto proj_faceA = -cylinder.support_projection(posA, ornA, -face_normals[i]);
            auto proj_faceB = dot(edge_vertices[0], face_normals[i]);
            distances_faces[i] = proj_faceA - proj_faceB;
        }

        auto distance = -EDYN_SCALAR_MAX;
        auto sep_axis = vector3_zero;

        /* Edge vs cylinder side edge. */ {
            auto dir = cross(cylinder_axis, edge_dir);

            if (try_normalize(dir)) {
                // Make it point towards cylinder.
                if (dot(posA - edge_vertices[0], dir) < 0) {
                    dir *= -1;
                }

                // Make dir point outside of mesh.
                if (dot(dir, face_normals[0]) < 0 &&
                    dot(dir, face_normals[1]) < 0) {
                    dir *= -1;
                }

                if (mesh.in_edge_voronoi(edge_idx, dir)) {
                    auto projA = -cylinder.support_projection(posA, ornA, -dir);
                    auto projB = dot(edge_vertices[0], dir);
                    auto edge_dist = projA - projB;

                    if (distances_faces[0] < edge_dist + EDYN_EPSILON &&
                        distances_faces[1] < edge_dist + EDYN_EPSILON &&
                        edge_dist > distance) {

                        distance = edge_dist;
                        sep_axis = dir;
                    }
                }
            }
        }

        // Edge against cylinder cap faces.
        for (auto i = 0; i < 2; ++i) {
            auto circle_position = cylinder_vertices[i];

            // Find closest points between circle and edge.
            size_t num_points;
            scalar s[2];
            vector3 closest_circle[2];
            vector3 closest_line[2];
            vector3 dir;
            closest_point_circle_line(circle_position, ornA, cylinder.radius,
                                      edge_vertices[0], edge_vertices[1], num_points,
                                      s[0], closest_circle[0], closest_line[0],
                                      s[1], closest_circle[1], closest_line[1],
                                      dir, support_feature_tolerance);

            // Make it point towards cylinder.
            if (dot(posA - edge_vertices[0], dir) < 0) {
                dir *= -1;
            }

            // Make dir point outside of mesh.
            if (dot(dir, face_normals[0]) < 0 &&
                dot(dir, face_normals[1]) < 0) {
                dir *= -1;
            }

            if (!mesh.in_edge_voronoi(edge_idx, dir)) {
                continue;
            }

            // Closest point direction is always orthogonal to edge line.
            EDYN_ASSERT(std::abs(dot(dir, normalize(edge_dir))) < 0.0001);

            auto projA = -cylinder.support_projection(posA, ornA, -dir);
            auto projB = dot(edge_vertices[0], dir);
            auto dist = projA - projB;

            if (distances_faces[0] < dist + EDYN_EPSILON &&
                distances_faces[1] < dist + EDYN_EPSILON &&
                dist > distance) {

                distance = dist;
                sep_axis = dir;
            }
        }

        if (distance > ctx.threshold || distance == -EDYN_SCALAR_MAX) {
            return;
        }

        cylinder_feature featureA;
        size_t feature_indexA;
        cylinder.support_feature(posA, ornA, -sep_axis,
                                 featureA, feature_indexA,
                                 support_feature_tolerance);

        switch (featureA) {
        case cylinder_feature::side_edge: {
            scalar s[2], t[2];
            vector3 closestA[2], closestB[2];
            size_t num_points = 0;
            closest_point_segment_segment(
                cylinder_vertices[0], cylinder_vertices[1],
                edge_vertices[0], edge_vertices[1],
                s[0], t[0], closestA[0], closestB[0], &num_points,
                &s[1], &t[1], &closestA[1], &closestB[1]);

            for (size_t i = 0; i < num_points; ++i) {
                if (!(s[i] > 0 && s[i] < 1 && t[i] > 0 && t[i] < 1)) continue;

                auto pivotA_world = closestA[i] - sep_axis * cylinder.radius;
                auto pivotA = to_object_space(pivotA_world, posA, ornA);
                auto pivotB = closestB[i];
                result.maybe_add_point({pivotA, pivotB, sep_axis, distance});
            }
            break;
        } case cylinder_feature::face: {
            // Check if edge intersects the circular cap.
            auto v0 = to_object_space(edge_vertices[0], posA, ornA);
            auto v1 = to_object_space(edge_vertices[1], posA, ornA);
            scalar s[2];

            auto num_points = intersect_line_circle(to_vector2_zy(v0), to_vector2_zy(v1),
                                                    cylinder.radius, s[0], s[1]);

            for (size_t i = 0; i < num_points; ++i) {
                auto t = s[i];
                if (!(t > 0 && t < 1)) continue;

                auto pivotB = lerp(edge_vertices[0], edge_vertices[1], t);
                auto pivotA_world = project_plane(pivotB, cylinder_vertices[feature_indexA], sep_axis);
                auto pivotA = to_object_space(pivotA_world, posA, ornA);
                auto local_distance = dot(pivotA_world - pivotB, sep_axis);
                result.add_point({pivotA, pivotB, sep_axis, local_distance});
            }
            break;
        } case cylinder_feature::cap_edge: {
            auto supportA = cylinder.support_point(posA, ornA, -sep_axis);
            vector3 closest; scalar t;
            closest_point_line(edge_vertices[0], edge_dir, supportA, t, closest);

            if (t > 0 && t < 1) {
                auto pivotA = to_object_space(supportA, posA, ornA);
                auto pivotB = closest;
                result.maybe_add_point({pivotA, pivotB, sep_axis, distance});
            }
        }}
    });

    mesh.visit_triangles(visit_aabb, [&] (auto tri_idx) {
        auto tri_vertices = mesh.get_triangle_vertices(tri_idx);
        auto tri_normal = mesh.get_triangle_normal(tri_idx);
        auto projA = -cylinder.support_projection(posA, ornA, -tri_normal);
        auto projB = dot(tri_vertices[0], tri_normal);
        auto distance = projA - projB;

        if (distance > ctx.threshold) {
            return;
        }

        cylinder_feature featureA;
        size_t feature_indexA;
        cylinder.support_feature(posA, ornA, -tri_normal,
                                 featureA, feature_indexA,
                                 support_feature_tolerance);

        switch (featureA) {
        case cylinder_feature::side_edge: {
            auto radial_dir = normalize(project_direction(-tri_normal, cylinder_axis));

            for (auto i = 0; i < 2; ++i) {
                auto pivotA_world = cylinder_vertices[i] + radial_dir * cylinder.radius;

                if (!point_in_triangle(tri_vertices, tri_normal, pivotA_world)) continue;

                auto pivotA = to_object_space(pivotA_world, posA, ornA);
                auto local_distance = dot(pivotA_world - tri_vertices[0], tri_normal);
                auto pivotB = pivotA_world - tri_normal * local_distance;
                result.maybe_add_point({pivotA, pivotB, tri_normal, local_distance});
            }
            break;
        } case cylinder_feature::face: {
            const auto multipliers = std::array<scalar, 4>{0, 1, 0, -1};

            for(int i = 0; i < 4; ++i) {
                auto j = (i + 1) % 4;
                auto pivotA_x = cylinder.half_length * to_sign(feature_indexA == 0);
                auto pivotA = vector3{pivotA_x,
                                      cylinder.radius * multipliers[i],
                                      cylinder.radius * multipliers[j]};
                auto pivotA_world = to_world_space(pivotA, posA, ornA);

                if (!point_in_triangle(tri_vertices, tri_normal, pivotA_world)) continue;

                auto local_distance = dot(pivotA_world - tri_vertices[0], tri_normal);
                auto pivotB = pivotA_world - tri_normal * local_distance;
                result.maybe_add_point({pivotA, pivotB, tri_normal, local_distance});
            }
            break;
        } case cylinder_feature::cap_edge: {
            auto supportA = cylinder.support_point(posA, ornA, -tri_normal);

            if (point_in_triangle(tri_vertices, tri_normal, supportA)) {
                auto pivotA = to_object_space(supportA, posA, ornA);
                auto pivotB = supportA - tri_normal * distance;
                result.maybe_add_point({pivotA, pivotB, tri_normal, distance});
            }
        }}
    });
}

}
