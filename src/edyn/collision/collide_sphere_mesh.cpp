#include "edyn/collision/collide.hpp"

namespace edyn {

collision_result collide(const sphere_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const mesh_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold) {
    auto result = collision_result{};

    // Sphere position in mesh's space.
    auto posA_in_B = rotate(conjugate(ornB), posA - posB);
    auto aabb = shA.aabb(posA_in_B, ornA); // Invariant to orientation.
    shB.trimesh->visit(aabb, [&] (const triangle_vertices &vertices) {
        if (result.num_points == max_contacts) {
            return;
        }
        
        auto edges = get_triangle_edges(vertices);
        auto normal = cross(edges[0], edges[1]);
        auto nl2 = length2(normal);

        if (nl2 < EDYN_EPSILON) {
            // Degenerate triangle.
            return;
        }

        normal /= std::sqrt(nl2);
        auto p = posA_in_B - vertices[0];
        auto dist_plane = dot(p, normal);

        if (dist_plane < 0) {
            // Flip triangle.
            dist_plane *= -1;
            normal *= -1;
        }

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

            for (size_t i = 0; i < 3; ++i) {
                auto v0 = vertices[i];
                auto v1 = vertices[(i + 1) % 3];
                vector3 q; scalar t;
                auto d2 = closest_point_segment(v0, v1, posA_in_B, t, q);

                if (d2 < dist_sqr) {
                    dist_sqr = d2;
                    closest_point = q;
                    has_contact = true;
                }
            }

            if (has_contact) {
                auto edge_normal = posA_in_B - closest_point;
                auto edge_normal_len = length(edge_normal);
                
                if (edge_normal_len > EDYN_EPSILON) {
                    edge_normal /= edge_normal_len;
                } else {
                    edge_normal = normal;
                }

                auto closest_point_world = posB + rotate(ornB, closest_point);
                auto idx = result.num_points++;
                result.point[idx].pivotA = rotate(conjugate(ornA), closest_point_world - posA);
                result.point[idx].pivotB = closest_point;
                result.point[idx].normalB = edge_normal;
                result.point[idx].distance = edge_normal_len - shA.radius;
            }
        }
    });

    return result;
}

}