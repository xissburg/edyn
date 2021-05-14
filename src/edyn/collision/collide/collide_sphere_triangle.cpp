#include "edyn/collision/collide.hpp"
#include "edyn/math/math.hpp"
#include "edyn/math/scalar.hpp"

namespace edyn {

void collide(const sphere_shape &sphere, const triangle_mesh &mesh, size_t tri_idx,
             const collision_context &ctx, collision_result &result) {
    const auto &sphere_pos = ctx.posA;
    const auto &sphere_orn = ctx.ornA;
    const auto threshold = ctx.threshold;

    auto tri_vertices = mesh.get_triangle_vertices(tri_idx);
    auto tri_normal = mesh.get_triangle_normal(tri_idx);

    auto dist_plane = dot(sphere_pos - tri_vertices[0], tri_normal);

    if (dist_plane > sphere.radius + threshold) {
        // Does not intersect triangle plane.
        return;
    }

    if (point_in_triangle(tri_vertices, tri_normal, sphere_pos)) {
        auto pivotA = rotate(conjugate(sphere_orn), -tri_normal * sphere.radius);
        auto pivotB = sphere_pos - tri_normal * dist_plane;
        auto normalB = tri_normal;
        auto distance = dist_plane - sphere.radius;
        result.add_point({pivotA, pivotB, normalB, distance});
    } else {
        // Check edges.
        const auto min_dist_sqr = square(sphere.radius + threshold);
        auto dist_sqr = min_dist_sqr;
        vector3 closest_point;
        vector3 closest_edge_normal;
        scalar closest_edge_dist;

        for (size_t i = 0; i < 3; ++i) {
            auto j = (i + 1) % 3;
            auto v0 = tri_vertices[i];
            auto v1 = tri_vertices[j];
            vector3 closest; scalar t;
            auto edge_dist_sqr = closest_point_segment(v0, v1, sphere_pos, t, closest);

            if (edge_dist_sqr > dist_sqr || !(edge_dist_sqr > EDYN_EPSILON)) {
                continue;
            }

            // Check Voronoi region.
            auto edge_normal = sphere_pos - closest;
            auto edge_idx = mesh.get_face_edge_index(tri_idx, i);
            auto in_voronoi_region = false;

            if (t < EDYN_EPSILON) {
                auto vertex_idx = mesh.get_face_vertex_index(tri_idx, i);
                in_voronoi_region = mesh.in_vertex_voronoi(vertex_idx, edge_normal);
            } else if (t > scalar(1) - EDYN_EPSILON) {
                auto vertex_idx = mesh.get_face_vertex_index(tri_idx, j);
                in_voronoi_region = mesh.in_vertex_voronoi(vertex_idx, edge_normal);
            } else {
                in_voronoi_region = mesh.in_edge_voronoi(edge_idx, edge_normal);
            }

            if (in_voronoi_region) {
                auto edge_dist = std::sqrt(edge_dist_sqr);
                edge_normal /= edge_dist;
                dist_sqr = edge_dist_sqr;
                closest_point = closest;
                closest_edge_normal = edge_normal;
                closest_edge_dist = edge_dist;
            }
        }

        if (dist_sqr < min_dist_sqr) {
            auto pivotA = to_object_space(sphere_pos - closest_edge_normal * sphere.radius,
                                          sphere_pos, sphere_orn);
            auto pivotB = closest_point;
            auto normalB = closest_edge_normal;
            auto distance = closest_edge_dist - sphere.radius;
            result.add_point({pivotA, pivotB, normalB, distance});
        }
    }
}

}
