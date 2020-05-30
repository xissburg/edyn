#include "edyn/collision/collide.hpp"

namespace edyn {

collision_result collide(const sphere_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const mesh_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold) {
    auto result = collision_result{};

    // Sphere position in mesh's space.
    auto posA_in_B = rotate(conjugate(ornB), posA - posB);
    auto aabb = shA.aabb(posA_in_B, ornA); // Invariant to orientation.
    shB.trimesh->visit(aabb, [&] (size_t tri_idx, const triangle_vertices &vertices) {
        if (result.num_points == max_contacts) {
            return;
        }
        
        auto edges = get_triangle_edges(vertices);
        auto normal = cross(edges[0], edges[1]);
        auto normal_len_sqr = length_sqr(normal);

        if (normal_len_sqr < EDYN_EPSILON) {
            // Degenerate triangle.
            return;
        }

        normal /= std::sqrt(normal_len_sqr);
        auto p = posA_in_B - vertices[0];
        auto dist_plane = dot(p, normal);

        if (dist_plane > shA.radius + threshold) {
            // Does not intersect triangle plane.
            return;
        }

        if (point_in_triangle(vertices, normal, posA_in_B)) {
            auto normal_world = rotate(ornB, normal);
            auto idx = result.num_points++;
            result.point[idx].pivotA = rotate(conjugate(ornA), -normal_world * shA.radius);
            result.point[idx].pivotB = posA_in_B - normal * dist_plane;
            result.point[idx].normalB = normal;
            result.point[idx].distance = dist_plane - shA.radius;
        } else {
            // Check edges.
            auto dist_sqr = shA.radius + threshold;
            dist_sqr *= dist_sqr;
            bool has_contact = false;
            vector3 closest_point;
            vector3 closest_edge_normal;
            scalar closest_edge_dist;

            for (size_t i = 0; i < 3; ++i) {
                // Ignore concave edges.
                if (shB.trimesh->is_concave_edge[tri_idx * 3 + i]) {
                    continue;
                }

                auto v0 = vertices[i];
                auto v1 = vertices[(i + 1) % 3];
                vector3 edge_point; scalar t;
                auto edge_dist_sqr = closest_point_segment(v0, v1, posA_in_B, t, edge_point);

                if (edge_dist_sqr < dist_sqr) {
                    // Check Voronoi region.
                    auto edge_normal = posA_in_B - edge_point;
                    auto edge_dist = std::sqrt(edge_dist_sqr);

                    if (edge_dist > EDYN_EPSILON) {
                        edge_normal /= edge_dist;
                    } else {
                        edge_normal = normal;
                    }
                    
                    // Check if angle between edge normal and the i-th triangle normal
                    // is in the range between the i-th and k-th triangle normals. 
                    if (dot(edge_normal, normal) > shB.trimesh->cos_angles[tri_idx * 3 + i]) {
                        dist_sqr = edge_dist_sqr;
                        closest_point = edge_point;
                        closest_edge_normal = edge_normal;
                        closest_edge_dist = edge_dist;
                        has_contact = true;
                    }
                }
            }

            if (has_contact) {
                auto pivotA_world = posB + rotate(ornB, posA_in_B - closest_edge_normal * shA.radius);
                auto idx = result.num_points++;
                result.point[idx].pivotA = rotate(conjugate(ornA), pivotA_world - posA);
                result.point[idx].pivotB = closest_point;
                result.point[idx].normalB = closest_edge_normal;
                result.point[idx].distance = closest_edge_dist - shA.radius;
            }
        }
    });

    return result;
}

}