#include "edyn/collision/collide.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/util/shape_util.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/math/math.hpp"

namespace edyn {

collision_result collide(const polyhedron_shape &shA, const capsule_shape &shB,
                         const collision_context &ctx) {
    const auto posA = vector3_zero;
    const auto &ornA = ctx.ornA;
    const auto posB = ctx.posB - ctx.posA;
    const auto &ornB = ctx.ornB;
    auto threshold = ctx.threshold;
    auto &rmeshA = ctx.rmeshA->get();

    auto capsule_axis = quaternion_x(ornB);
    auto capsule_vertices = std::array<vector3, 2>{
        posB - capsule_axis * shB.half_length,
        posB + capsule_axis * shB.half_length
    };

    scalar distance = -EDYN_SCALAR_MAX;
    scalar projection_poly = EDYN_SCALAR_MAX;
    auto sep_axis = vector3_zero;

    // Face normals of polyhedron.
    for (size_t i = 0; i < shA.mesh->num_faces(); ++i) {
        auto normal_world = -rmeshA.normals[i]; // Point towards polyhedron.
        auto vertex_idx = shA.mesh->first_vertex_index(i);
        auto &vertex_world = rmeshA.vertices[vertex_idx];

        auto projA = dot(vertex_world, normal_world);
        auto projB = capsule_support_projection(capsule_vertices, shB.radius, normal_world);
        auto dist = projA - projB;

        if (dist > distance) {
            distance = dist;
            projection_poly = projA;
            sep_axis = normal_world;
        }
    }

    // Edges vs capsule axis
    for (size_t i = 0; i < shA.mesh->num_edges(); ++i) {
        auto [vertexA0, vertexA1] = shA.mesh->get_edge(rmeshA, i);
        scalar s, t;
        vector3 closestA, closestB;
        closest_point_segment_segment(vertexA0, vertexA1, 
                                      capsule_vertices[0], capsule_vertices[1], 
                                      s, t, closestA, closestB);
        auto dir = closestA - closestB;
        auto dir_len_sqr = length_sqr(dir);

        if (!(dir_len_sqr > EDYN_EPSILON)) {
            continue;
        }

        dir /= std::sqrt(dir_len_sqr);

        if (dot(posA - posB, dir) < 0) {
            // Make it point towards A.
            dir *= -1;
        }

        auto projA = -point_cloud_support_projection(rmeshA.vertices, -dir);
        auto projB = capsule_support_projection(capsule_vertices, shB.radius, dir);
        auto dist = projA - projB;

        if (dist > distance) {
            distance = dist;
            projection_poly = projA;
            sep_axis = dir;
        }
    }

    if (distance > threshold) {
        return {};
    }

    auto result = collision_result{};
    auto normalB = rotate(conjugate(ornB), sep_axis);

    scalar proj_capsule_vertices[] = {
        dot(capsule_vertices[0], sep_axis),
        dot(capsule_vertices[1], sep_axis)
    };

    auto is_capsule_edge = std::abs(proj_capsule_vertices[0] -
                                    proj_capsule_vertices[1]) < threshold;

    if (is_capsule_edge) {
        auto polygon = point_cloud_support_polygon<true>(
            rmeshA.vertices.begin(), rmeshA.vertices.end(), vector3_zero,
            sep_axis, projection_poly, true, support_polygon_tolerance);

        // Check if the vertices of the capsule are inside the polygon.
        if (polygon.hull.size() > 2) {
            for (auto &pointB : capsule_vertices) {
                if (point_in_polygonal_prism(polygon.vertices, polygon.hull, sep_axis, pointB)) {
                    auto pivotB = to_object_space(pointB + sep_axis * shB.radius, posB, ornB);
                    auto pivotA_world = project_plane(pointB, polygon.origin, sep_axis);
                    auto pivotA = to_object_space(pivotA_world, posA, ornA);
                    result.add_point({pivotA, pivotB, normalB, distance});
                }
            }
        }

        // Do not continue if there are already 2 points in the result, which means
        // both vertices of the capsule are contained in the polygon.
        if (result.num_points == 2) {
            return result;
        }
        
        // Check if the capsule edge intersects the polygon's edges. 
        if (polygon.hull.size() > 1) {
            // If the feature is a polygon, it will be necessary to wrap around the 
            // vertex array. If it is just one edge, then avoid calculating the same
            // segment-segment intersection twice.
            const auto sizeA = polygon.hull.size();
            const auto limitA = sizeA == 2 ? 1 : sizeA;
            scalar s[2], t[2];

            vector2 plane_capsule_vertices[] = {
                to_vector2_xz(to_object_space(capsule_vertices[0], polygon.origin, polygon.basis)),
                to_vector2_xz(to_object_space(capsule_vertices[1], polygon.origin, polygon.basis))
            };

            for (size_t i = 0; i < limitA; ++i) {
                auto idx0A = polygon.hull[i];
                auto idx1A = polygon.hull[(i + 1) % sizeA];
                auto &v0A = polygon.plane_vertices[idx0A];
                auto &v1A = polygon.plane_vertices[idx1A];

                auto num_points = intersect_segments(v0A, v1A, plane_capsule_vertices[0], plane_capsule_vertices[1], 
                                                     s[0], t[0], s[1], t[1]);

                for (size_t k = 0; k < num_points; ++k) {
                    auto pivotA_world = lerp(polygon.vertices[idx0A], polygon.vertices[idx1A], s[k]);
                    auto pivotB_world = lerp(capsule_vertices[0], capsule_vertices[1], t[k]) + sep_axis * shB.radius;
                    auto pivotA = to_object_space(pivotA_world, posA, ornA);
                    auto pivotB = to_object_space(pivotB_world, posB, ornB);
                    result.maybe_add_point({pivotA, pivotB, normalB, distance});
                }
            }
        } else {
            // Polyhedron vertex against capsule edge.
            EDYN_ASSERT(polygon.hull.size() == 1);
            auto &pivotA_world = polygon.vertices[polygon.hull[0]];
            auto edge_dir = capsule_vertices[1] - capsule_vertices[0];
            vector3 pivotB_world; scalar t;
            closest_point_line(capsule_vertices[0], edge_dir, pivotA_world, t, pivotB_world);
            auto pivotB = to_object_space(pivotB_world, posB, ornB) + normalB * shB.radius;
            auto pivotA = to_object_space(pivotA_world, posA, ornA);
            result.add_point({pivotA, pivotB, normalB, distance});
        }
    } else {
        auto &closest_capsule_vertex = proj_capsule_vertices[0] > proj_capsule_vertices[1] ? 
                                       capsule_vertices[0] : capsule_vertices[1];
        auto pivotB_world = closest_capsule_vertex + sep_axis * shB.radius;
        auto pivotA_world = pivotB_world + sep_axis * distance;
        auto pivotB = to_object_space(pivotB_world, posB, ornB);
        auto pivotA = to_object_space(pivotA_world, posA, ornA);
        result.add_point({pivotA, pivotB, normalB, distance});
    }

    return result;
}

collision_result collide(const capsule_shape &shA, const polyhedron_shape &shB,
                         const collision_context &ctx) {
    return swap_collide(shA, shB, ctx);
}

}
