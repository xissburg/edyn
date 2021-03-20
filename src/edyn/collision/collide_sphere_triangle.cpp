#include "edyn/collision/collide.hpp"
#include "edyn/shapes/triangle_shape.hpp"
#include "edyn/math/math.hpp"
#include <algorithm>

namespace edyn {

void collide_sphere_triangle(
    const sphere_shape &sphere, 
    const vector3 &sphere_pos,
    const quaternion &sphere_orn,
    const triangle_vertices &vertices, 
    const std::array<bool, 3> &is_concave_edge, 
    const std::array<scalar, 3> &cos_angles,
    scalar threshold, collision_result &result) {

    auto edges = get_triangle_edges(vertices);
    auto normal = cross(edges[0], edges[1]);
    auto normal_len_sqr = length_sqr(normal);

    if (normal_len_sqr < EDYN_EPSILON) {
        // Degenerate triangle.
        return;
    }

    normal /= std::sqrt(normal_len_sqr);
    auto p = sphere_pos - vertices[0];
    auto dist_plane = dot(p, normal);

    if (dist_plane > sphere.radius + threshold) {
        // Does not intersect triangle plane.
        return;
    }

    if (point_in_triangle(vertices, normal, sphere_pos)) {
        auto idx = result.num_points++;
        result.point[idx].pivotA = rotate(conjugate(sphere_orn), -normal * sphere.radius);
        result.point[idx].pivotB = sphere_pos - normal * dist_plane;
        result.point[idx].normalB = normal;
        result.point[idx].distance = dist_plane - sphere.radius;
    } else {
        // Check edges.
        auto dist_sqr = sphere.radius + threshold;
        dist_sqr *= dist_sqr;
        bool has_contact = false;
        vector3 closest_point;
        vector3 closest_edge_normal;
        scalar closest_edge_dist;

        for (size_t i = 0; i < 3; ++i) {
            // Ignore concave edges.
            if (is_concave_edge[i]) {
                continue;
            }

            auto v0 = vertices[i];
            auto v1 = vertices[(i + 1) % 3];
            vector3 edge_point; scalar t;
            auto edge_dist_sqr = closest_point_segment(v0, v1, sphere_pos, t, edge_point);

            if (edge_dist_sqr < dist_sqr) {
                // Check Voronoi region.
                auto edge_normal = sphere_pos - edge_point;
                auto edge_dist = std::sqrt(edge_dist_sqr);

                if (edge_dist > EDYN_EPSILON) {
                    edge_normal /= edge_dist;
                } else {
                    edge_normal = normal;
                }
                
                // Check if angle between edge normal and the i-th triangle normal
                // is in the range between the i-th and k-th triangle normals. 
                if (dot(edge_normal, normal) > cos_angles[i]) {
                    dist_sqr = edge_dist_sqr;
                    closest_point = edge_point;
                    closest_edge_normal = edge_normal;
                    closest_edge_dist = edge_dist;
                    has_contact = true;
                }
            }
        }

        if (has_contact) {
            auto pivotA = to_object_space(sphere_pos - closest_edge_normal * sphere.radius, 
                                          sphere_pos, sphere_orn);
            auto idx = result.num_points++;
            result.point[idx].pivotA = pivotA;
            result.point[idx].pivotB = closest_point;
            result.point[idx].normalB = closest_edge_normal;
            result.point[idx].distance = closest_edge_dist - sphere.radius;
        }
    }
}

}
