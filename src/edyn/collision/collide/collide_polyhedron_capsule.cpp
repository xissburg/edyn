#include "edyn/collision/collide.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/util/shape_util.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/math/math.hpp"
#include "edyn/math/transform.hpp"

namespace edyn {

void collide(const polyhedron_shape &shA, const capsule_shape &shB,
             const collision_context &ctx, collision_result &result) {
    // Convex polyhedron vs capsule SAT. All calculations done in the
    // polyhedron's space.
    const auto posB = to_object_space(ctx.posB, ctx.posA, ctx.ornA);
    const auto ornB = conjugate(ctx.ornA) * ctx.ornB;
    auto threshold = ctx.threshold;
    auto &meshA = *shA.mesh;

    auto capsule_vertices = shB.get_vertices(posB, ornB);

    scalar distance = -EDYN_SCALAR_MAX;
    scalar projection_poly = EDYN_SCALAR_MAX;
    auto sep_axis = vector3_zero;

    // Face normals of polyhedron.
    for (auto face_idx : meshA.relevant_faces) {
        auto normalA = -meshA.normals[face_idx]; // Point towards polyhedron.
        auto vertexA = meshA.vertices[meshA.first_vertex_index(face_idx)];

        auto projA = dot(vertexA, normalA);
        auto projB = capsule_support_projection(capsule_vertices, shB.radius, normalA);
        auto dist = projA - projB;

        if (dist > distance) {
            distance = dist;
            projection_poly = projA;
            sep_axis = normalA;
        }
    }

    // Edges vs capsule axis
    for (size_t i = 0; i < meshA.num_edges(); ++i) {
        auto [vertexA0, vertexA1] = meshA.get_edge(i);
        scalar s, t;
        vector3 closestA, closestB;
        closest_point_segment_segment(vertexA0, vertexA1,
                                      capsule_vertices[0], capsule_vertices[1],
                                      s, t, closestA, closestB);
        auto dir = closestA - closestB;

        if (!try_normalize(dir)) {
            continue;
        }

        if (dot(posB, dir) > 0) {
            // Make it point towards A.
            dir *= -1;
        }

        auto projA = -polyhedron_support_projection(meshA.vertices, meshA.neighbors_start, meshA.neighbor_indices, -dir);
        auto projB = capsule_support_projection(capsule_vertices, shB.radius, dir);
        auto dist = projA - projB;

        if (dist > distance) {
            distance = dist;
            projection_poly = projA;
            sep_axis = dir;
        }
    }

    if (distance > threshold) {
        return;
    }

    scalar proj_capsule_vertices[] = {
        dot(capsule_vertices[0], sep_axis),
        dot(capsule_vertices[1], sep_axis)
    };

    auto is_capsule_edge = std::abs(proj_capsule_vertices[0] -
                                    proj_capsule_vertices[1]) < support_feature_tolerance;

    auto polygon = point_cloud_support_polygon(
        meshA.vertices.begin(), meshA.vertices.end(), vector3_zero,
        sep_axis, projection_poly, true, support_feature_tolerance);

    collision_result::collision_point point;
    // Separating axis is in A's space. Transform it to world space.
    point.normal = rotate(ctx.ornA, sep_axis);
    point.distance = distance;
    point.normal_attachment = polygon.hull.size() > 2 ?
            contact_normal_attachment::normal_on_A:
            contact_normal_attachment::none;

    if (is_capsule_edge) {
        point.featureB = {capsule_feature::side};

        // Check if the vertices of the capsule are inside the polygon.
        if (polygon.hull.size() > 2) {
            for (auto &pointB : capsule_vertices) {
                if (point_in_polygonal_prism(polygon.vertices, polygon.hull, sep_axis, pointB)) {
                    point.pivotA = project_plane(pointB, polygon.origin, sep_axis);
                    point.pivotB = to_object_space(pointB + sep_axis * shB.radius, posB, ornB);
                    result.add_point(point);
                }
            }
        }

        // Do not continue if there are already 2 points in the result, which means
        // both vertices of the capsule are contained in the polygon.
        if (result.num_points == 2) {
            return;
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
                    point.pivotA = lerp(polygon.vertices[idx0A], polygon.vertices[idx1A], s[k]);
                    auto pivotB_world = lerp(capsule_vertices[0], capsule_vertices[1], t[k]) + sep_axis * shB.radius;
                    point.pivotB = to_object_space(pivotB_world, posB, ornB);
                    result.maybe_add_point(point);
                }
            }
        } else {
            // Polyhedron vertex against capsule edge.
            EDYN_ASSERT(polygon.hull.size() == 1);
            point.pivotA = polygon.vertices[polygon.hull[0]];
            auto edge_dir = capsule_vertices[1] - capsule_vertices[0];
            vector3 pivotB_world; scalar t;
            closest_point_line(capsule_vertices[0], edge_dir, point.pivotA, t, pivotB_world);
            auto normalB = rotate(conjugate(ornB), sep_axis);
            point.pivotB = to_object_space(pivotB_world, posB, ornB) + normalB * shB.radius;
            result.add_point(point);
        }
    } else {
        size_t closest_capsule_vertex_index = proj_capsule_vertices[0] > proj_capsule_vertices[1] ? 0 : 1;
        auto &closest_capsule_vertex = capsule_vertices[closest_capsule_vertex_index];
        auto pivotB_world = closest_capsule_vertex + sep_axis * shB.radius;
        point.pivotB = to_object_space(pivotB_world, posB, ornB);
        point.pivotA = pivotB_world + sep_axis * distance;
        point.featureB = {capsule_feature::hemisphere, closest_capsule_vertex_index};
        result.add_point(point);
    }
}

void collide(const capsule_shape &shA, const polyhedron_shape &shB,
             const collision_context &ctx, collision_result &result) {
    swap_collide(shA, shB, ctx, result);
}

}
