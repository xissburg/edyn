#include "edyn/collision/collide.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/math/math.hpp"
#include "edyn/math/scalar.hpp"
#include "edyn/math/vector3.hpp"
#include <cmath>

namespace edyn {

void collide(const sphere_shape &sphere, const triangle_mesh &mesh,
             const collision_context &ctx, collision_result &result) {
    const auto &sphere_pos = ctx.posA;
    const auto &sphere_orn = ctx.ornA;

    const auto inset = vector3_one * -contact_breaking_threshold;
    const auto visit_aabb = ctx.aabbA.inset(inset);

    mesh.visit_vertices(visit_aabb, [&] (auto vertex_idx) {
        if (mesh.is_concave_vertex(vertex_idx)) return;

        auto vertex = mesh.get_vertex_position(vertex_idx);
        auto dir = sphere_pos - vertex;
        auto dir_len_sqr = length_sqr(dir);
        auto min_dist = sphere.radius + ctx.threshold;

        if (dir_len_sqr < EDYN_EPSILON || dir_len_sqr > min_dist * min_dist) {
            return;
        }

        if (!mesh.in_vertex_voronoi(vertex_idx, dir)) {
            return;
        }

        dir /= std::sqrt(dir_len_sqr);

        auto pivotA = rotate(conjugate(sphere_orn), -dir * sphere.radius);
        auto pivotB = vertex;
        auto distance = dot(sphere_pos - vertex, dir) - sphere.radius;
        result.maybe_add_point({pivotA, pivotB, dir, distance});
    });

    mesh.visit_edges(visit_aabb, [&] (auto edge_idx) {
        if (mesh.is_concave_edge(edge_idx)) {
            return;
        }

        auto edge_vertices = mesh.get_edge_vertices(edge_idx);
        auto edge_dir = edge_vertices[1] - edge_vertices[0];

        vector3 closest; scalar t;
        closest_point_line(edge_vertices[0], edge_dir, sphere_pos, t, closest);

        if (!(t > 0 && t < 1)) {
            return;
        }

        auto dir = sphere_pos - closest;
        auto dir_len_sqr = length_sqr(dir);
        auto min_dist = sphere.radius + ctx.threshold;

        if (dir_len_sqr < EDYN_EPSILON || dir_len_sqr > min_dist * min_dist) {
            return;
        }

        auto dir_len = std::sqrt(dir_len_sqr);
        dir /= dir_len;

        // Flip direction if the sphere center is behind both faces that share
        // this edge.
        auto face_normals = mesh.get_convex_edge_face_normals(edge_idx);

        if (dot(dir, face_normals[0]) < 0 &&
            dot(dir, face_normals[1]) < 0) {
            dir *= -1;
        }

        if (!mesh.in_edge_voronoi(edge_idx, dir)) {
            return;
        }

        auto pivotA = rotate(conjugate(sphere_orn), -dir * sphere.radius);
        auto pivotB = closest;
        auto distance = dot(sphere_pos - closest, dir) - sphere.radius;
        result.maybe_add_point({pivotA, pivotB, dir, distance});
    });

    mesh.visit_triangles(visit_aabb, [&] (auto tri_idx) {
        auto tri_vertices = mesh.get_triangle_vertices(tri_idx);
        auto tri_normal = mesh.get_triangle_normal(tri_idx);

        auto dist_plane = dot(sphere_pos - tri_vertices[0], tri_normal);
        auto distance = dist_plane - sphere.radius;

        if (distance > ctx.threshold) {
            return;
        }

        if (point_in_triangle(tri_vertices, tri_normal, sphere_pos)) {
            auto pivotA = rotate(conjugate(sphere_orn), -tri_normal * sphere.radius);
            auto pivotB = sphere_pos - tri_normal * dist_plane;
            result.maybe_add_point({pivotA, pivotB, tri_normal, distance});
        }
    });
}

}
