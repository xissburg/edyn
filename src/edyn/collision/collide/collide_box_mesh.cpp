#include "edyn/collision/collide.hpp"
#include "edyn/config/constants.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/math/math.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/math/vector2_3_util.hpp"

namespace edyn {

void collide(const box_shape &box, const triangle_mesh &mesh,
             const collision_context &ctx, collision_result &result) {
    const auto &posA = ctx.posA;
    const auto &ornA = ctx.ornA;

    const auto box_axes = std::array<vector3, 3> {
        quaternion_x(ctx.ornA),
        quaternion_y(ctx.ornA),
        quaternion_z(ctx.ornA)
    };

    const vector3 box_dir_local =
        vector3_x * box.half_extents[0] +
        vector3_y * box.half_extents[1] +
        vector3_z * box.half_extents[2];

    const auto box_vertices_local = std::array<vector3, 8> {
        box_dir_local * vector3{ 1,  1,  1},
        box_dir_local * vector3{-1,  1,  1},
        box_dir_local * vector3{-1, -1,  1},
        box_dir_local * vector3{ 1, -1,  1},
        box_dir_local * vector3{ 1,  1, -1},
        box_dir_local * vector3{-1,  1, -1},
        box_dir_local * vector3{-1, -1, -1},
        box_dir_local * vector3{ 1, -1, -1},
    };

    auto rotA = matrix3x3_columns(box_axes[0], box_axes[1], box_axes[2]);

    auto box_vertices = std::array<vector3, 8>{};
    for (size_t i = 0; i < box_vertices.size(); ++i) {
        box_vertices[i] = to_world_space(box_vertices_local[i], posA, rotA);
    }

    const auto inset = vector3_one * -contact_breaking_threshold;
    const auto visit_aabb = ctx.aabbA.inset(inset);

    mesh.visit_vertices(visit_aabb, [&] (auto vertex_idx) {
        if (mesh.is_concave_vertex(vertex_idx)) return;

        auto vertex = mesh.get_vertex_position(vertex_idx);
        auto d = vertex - posA;
        // The projection of d onto the box_axes represent the vertex position
        // in the box'es object space.
        auto proj = vector3{dot(d, box_axes[0]), dot(d, box_axes[1]), dot(d, box_axes[2])};

        if (abs(proj) < box.half_extents) {
            // Vertex is inside box.
            vector3 closest, normal;
            auto distance = closest_point_box_inside(box.half_extents, proj, closest, normal);
            // Make normal point towards box in world space.
            normal = rotate(ornA, -normal);

            if (!mesh.in_vertex_voronoi(vertex_idx, normal)) {
                return;
            }

            auto pivotA = closest;
            auto pivotB = vertex;
            result.maybe_add_point({pivotA, pivotB, normal, -distance});
        } else {
            for (size_t i = 0; i < 3; ++i) {
                auto j = (i + 1) % 3;
                auto k = (i + 2) % 3;

                if (std::abs(proj[j]) < box.half_extents[j] &&
                    std::abs(proj[k]) < box.half_extents[k]) {
                    auto distance = std::abs(proj[i]) - box.half_extents[i];

                    if (distance > ctx.threshold) {
                        return;
                    }

                    auto sign = proj[i] > 0 ? scalar(1) : scalar(-1);
                    auto normal = box_axes[i] * -sign;

                    if (!mesh.in_vertex_voronoi(vertex_idx, normal)) {
                        return;
                    }

                    auto pivotA = proj;
                    pivotA[i] = sign * box.half_extents[i];
                    auto pivotB = vertex;
                    result.maybe_add_point({pivotA, pivotB, normal, distance});

                    break;
                }
            }
        }
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
        // This is a necessary check for a situation where the box touches one
        // of the faces and this edge has a sharp angle (generally greater than
        // 90 degrees) which would cause it to shoot towards the edge since a
        // very deep intersection would be considered with a direction that is
        // within the Voronoi region. This test could be eliminated if it could
        // be assumed that the mesh does not have sharp angles.
        auto distances_faces = std::array<scalar, face_normals.size()>{};

        for (auto i = 0; i < face_normals.size(); ++i) {
            auto proj_faceA = -box.support_projection(posA, ornA, -face_normals[i]);
            auto proj_faceB = dot(edge_vertices[0], face_normals[i]);
            distances_faces[i] = proj_faceA - proj_faceB;
        }

        auto distance = -EDYN_SCALAR_MAX;
        auto sep_axis = vector3_zero;

        for (auto axis : box_axes) {
            auto dir = cross(axis, edge_dir);

            if (!try_normalize(dir)) {
                continue;
            }

            // Make it point towards box.
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

            auto projA = -box.support_projection(posA, ornA, -dir);
            auto projB = dot(edge_vertices[0], dir);
            auto edge_dist = projA - projB;

            if (distances_faces[0] > edge_dist + EDYN_EPSILON ||
                distances_faces[1] > edge_dist + EDYN_EPSILON) {
                continue;
            }

            if (edge_dist > distance) {
                distance = edge_dist;
                sep_axis = dir;
            }
        }

        if (distance > ctx.threshold || distance == -EDYN_SCALAR_MAX) {
            return;
        }

        box_feature featureA;
        size_t feature_indexA;
        box.support_feature(posA, ornA, -sep_axis,
                            featureA, feature_indexA,
                            support_feature_tolerance);

        if (featureA == box_feature::edge) {
            auto box_edge = box.get_edge(feature_indexA, posA, ornA);
            auto box_edge_local = box.get_edge(feature_indexA);

            scalar s[2], t[2];
            vector3 closestA[2], closestB[2];
            size_t num_points = 0;
            closest_point_segment_segment(
                box_edge[0], box_edge[1], edge_vertices[0], edge_vertices[1],
                s[0], t[0], closestA[0], closestB[0], &num_points,
                &s[1], &t[1], &closestA[1], &closestB[1]);

            for (size_t i = 0; i < num_points; ++i) {
                if (!(s[i] > 0 && s[i] < 1 && t[i] > 0 && t[i] < 1)) continue;

                auto pivotA = lerp(box_edge_local[0], box_edge_local[1], s[i]);
                auto pivotB = closestB[i];
                result.maybe_add_point({pivotA, pivotB, sep_axis, distance});
            }
        } else if (featureA == box_feature::face) {
            auto normalA = box.get_face_normal(feature_indexA, ornA);
            auto verticesA = box.get_face(feature_indexA, posA, ornA);

            auto e0_in_A = to_object_space(edge_vertices[0], posA, ornA);
            auto e1_in_A = to_object_space(edge_vertices[1], posA, ornA);

            vector2 half_extents;
            vector2 p0, p1;

            if (feature_indexA == 0 || feature_indexA == 1) { // X face
                half_extents = to_vector2_zy(box.half_extents);
                p0 = to_vector2_zy(e0_in_A); p1 = to_vector2_zy(e1_in_A);
            } else if (feature_indexA == 2 || feature_indexA == 3) { // Y face
                half_extents = to_vector2_xz(box.half_extents);
                p0 = to_vector2_xz(e0_in_A); p1 = to_vector2_xz(e1_in_A);
            } else if (feature_indexA == 4 || feature_indexA == 5) { // Z face
                half_extents = to_vector2_xy(box.half_extents);
                p0 = to_vector2_xy(e0_in_A); p1 = to_vector2_xy(e1_in_A);
            } else {
                EDYN_ASSERT(false);
            }

            scalar s[2];
            auto num_points = intersect_line_aabb(p0, p1, -half_extents, half_extents, s[0], s[1]);

            for (size_t k = 0; k < num_points; ++k) {
                if (s[k] < 0 || s[k] > 1) continue;

                auto pivotB = lerp(edge_vertices[0], edge_vertices[1], s[k]);
                auto pivotA_world = project_plane(pivotB, verticesA[0], normalA);
                auto pivotA = to_object_space(pivotA_world, posA, ornA);
                auto local_distance = dot(pivotA_world - pivotB, sep_axis);
                result.maybe_add_point({pivotA, pivotB, sep_axis, local_distance});
            }
        }
    });

    mesh.visit_triangles(visit_aabb, [&] (auto tri_idx) {
        auto tri_vertices = mesh.get_triangle_vertices(tri_idx);
        auto tri_normal = mesh.get_triangle_normal(tri_idx);
        auto projA = -box.support_projection(posA, ornA, -tri_normal);
        auto projB = dot(tri_vertices[0], tri_normal);
        auto distance = projA - projB;

        if (distance > ctx.threshold) {
            return;
        }

        for (size_t i = 0; i < box_vertices.size(); ++i) {
            auto box_vertex = box_vertices[i];
            auto proj_vertex = dot(box_vertex, tri_normal);

            if (proj_vertex > projA + support_feature_tolerance) {
                continue;
            }

            if (point_in_triangle(tri_vertices, tri_normal, box_vertex)) {
                auto pivotA = box_vertices_local[i];
                auto pivotB = project_plane(box_vertex, tri_vertices[0], tri_normal);
                auto local_distance = dot(box_vertex - tri_vertices[0], tri_normal);
                result.maybe_add_point({pivotA, pivotB, tri_normal, local_distance});
            }
        }
    });
}

}
