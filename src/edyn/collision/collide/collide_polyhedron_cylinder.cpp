#include "edyn/collision/collide.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/math/vector2_3_util.hpp"
#include "edyn/math/transform.hpp"
#include "edyn/shapes/cylinder_shape.hpp"
#include "edyn/util/shape_util.hpp"
#include "edyn/math/math.hpp"
#include "edyn/math/constants.hpp"

namespace edyn {

void collide(const polyhedron_shape &shA, const cylinder_shape &shB,
             const collision_context &ctx, collision_result &result) {
    // Convex polyhedron against cylinder SAT. All calculations done in the
    // polyhedron's space.
    const auto posB = to_object_space(ctx.posB, ctx.posA, ctx.ornA);
    const auto ornB = conjugate(ctx.ornA) * ctx.ornB;
    const auto threshold = ctx.threshold;
    const auto &meshA = *shA.mesh;

    const auto cyl_axis = coordinate_axis_vector(shB.axis, ornB);
    const auto face_center_pos = posB + cyl_axis * shB.half_length;
    const auto face_center_neg = posB - cyl_axis * shB.half_length;

    scalar distance = -EDYN_SCALAR_MAX;
    scalar projection_poly = EDYN_SCALAR_MAX;
    auto sep_axis = vector3_zero;

    // Face normals of polyhedron.
    for (auto face_idx : meshA.relevant_faces) {
        auto normalA = -meshA.normals[face_idx]; // Point towards polyhedron.
        auto vertexA = meshA.vertices[meshA.first_vertex_index(face_idx)];

        // Find point on box that's furthest along the opposite direction
        // of the face normal.
        auto projA = dot(vertexA, normalA);
        auto projB = shB.support_projection(posB, ornB, normalA);
        auto dist = projA - projB;

        if (dist > distance) {
            distance = dist;
            projection_poly = projA;
            sep_axis = normalA;
        }
    }

    // Cylinder cap face normals.
    for (size_t i = 0; i < 2; ++i) {
        auto dir = std::array<vector3, 2>{cyl_axis, -cyl_axis}[i];
        auto projA = -polyhedron_support_projection(meshA.vertices, meshA.neighbors_start, meshA.neighbor_indices, -dir);
        auto projB = dot(posB, dir) + shB.half_length;
        auto dist = projA - projB;

        if (dist > distance) {
            distance = dist;
            projection_poly = projA;
            sep_axis = dir;
        }
    }

    // Polyhedron edges vs cylinder side edges.
    for (auto edge_idx : meshA.relevant_edges) {
        auto poly_edge = meshA.get_edge_direction(edge_idx);
        auto dir = cross(poly_edge, cyl_axis);

        if (!try_normalize(dir)) {
            continue;
        }

        if (dot(posB, dir) > 0) {
            // Make it point towards A.
            dir *= -1;
        }

        auto projA = -polyhedron_support_projection(meshA.vertices, meshA.neighbors_start, meshA.neighbor_indices, -dir);
        auto projB = shB.support_projection(posB, ornB, dir);
        auto dist = projA - projB;

        if (dist > distance) {
            distance = dist;
            projection_poly = projA;
            sep_axis = dir;
        }
    }

    // Polyhedron vertices vs cylinder side edges.
    for (auto &rvertex : meshA.vertices) {
        vector3 closest; scalar t;
        closest_point_line(face_center_neg, cyl_axis, rvertex, t, closest);
        auto dir = rvertex - closest;

        if (!try_normalize(dir)) {
            continue;
        }

        if (dot(posB, dir) > 0) {
            // Make it point towards A.
            dir *= -1;
        }

        auto projA = -polyhedron_support_projection(meshA.vertices, meshA.neighbors_start, meshA.neighbor_indices, -dir);
        auto projB = shB.support_projection(posB, ornB, dir);
        auto dist = projA - projB;

        if (dist > distance) {
            distance = dist;
            projection_poly = projA;
            sep_axis = dir;
        }
    }

    // Cylinder cap edges vs polyhedron edges.
    for (size_t i = 0; i < meshA.num_edges(); ++i) {
        auto [vertexA0, vertexA1] = meshA.get_edge(i);

        // Find closest point between circle and edge.
        for (size_t j = 0; j < 2; ++j) {
            auto face_center = j == 0 ? face_center_neg : face_center_pos;
            size_t num_points;
            scalar s0, s1;
            vector3 cc0, cl0, cc1, cl1;
            vector3 dir;
            closest_point_circle_line(face_center, ornB, shB.radius, shB.axis,
                                      vertexA0, vertexA1,
                                      num_points, s0, cc0, cl0, s1, cc1, cl1,
                                      dir, support_feature_tolerance);

            // Ignore edges that are parallel to circle. Those would be handled
            // by the cylinder cap face normals.
            if (num_points == 2) continue;

            // Ignore points out of the segment range.
            if (!(s0 > 0 && s0 < 1)) continue;

            if (dot(posB, dir) > 0) {
                // Make it point towards A.
                dir *= -1;
            }

            auto projA = -polyhedron_support_projection(meshA.vertices, meshA.neighbors_start, meshA.neighbor_indices, -dir);
            auto projB = shB.support_projection(posB, ornB, dir);
            auto dist = projA - projB;

            if (dist > distance) {
                distance = dist;
                projection_poly = projA;
                sep_axis = dir;
            }
        }
    }

    if (distance > threshold) {
        return;
    }

    // Separating axis is in A's space.
    auto normal = rotate(ctx.ornA, sep_axis);

    auto polygon = point_cloud_support_polygon(
        meshA.vertices.begin(), meshA.vertices.end(), vector3_zero,
        sep_axis, projection_poly, true, support_feature_tolerance);

    cylinder_feature featureB;
    size_t feature_indexB;
    shB.support_feature(posB, ornB, sep_axis, featureB, feature_indexB,
                        support_feature_tolerance);

    collision_result::collision_point point;
    point.normal = normal;
    point.distance = distance;
    point.featureB = {featureB, feature_indexB};

    auto cyl_ax_idx = static_cast<std::underlying_type_t<coordinate_axis>>(shB.axis);
    auto cyl_ax_idx_ortho0 = (cyl_ax_idx + 1) % 3;
    auto cyl_ax_idx_ortho1 = (cyl_ax_idx + 2) % 3;

    switch (featureB) {
    case cylinder_feature::face: {
        size_t num_vertices_in_face = 0;
        auto sign_faceB = to_sign(feature_indexB == 0);
        auto pivotB_axis = shB.half_length * sign_faceB;
        point.normal_attachment = contact_normal_attachment::normal_on_B;

        // Check if polygon vertices are inside a cylinder cap face (by checking
        // if its distance from the cylinder axis is smaller than the cylinder radius).
        for (auto idxA : polygon.hull) {
            auto &pointA = polygon.vertices[idxA];
            vector3 closest; scalar t;
            auto dist_sqr = closest_point_line(posB, cyl_axis, pointA, t, closest);

            if (dist_sqr > shB.radius * shB.radius) continue;

            point.pivotA = pointA;
            point.pivotB = to_object_space(pointA, posB, ornB);
            point.pivotB[cyl_ax_idx] = pivotB_axis;
            result.maybe_add_point(point);

            ++num_vertices_in_face;
        }

        // If not all vertices of the polygon are contained in the cylinder
        // cap face, there could be edge intersections or the cylinder cap
        // face could be contained within the polygon.
        size_t num_edge_intersections = 0;

        const auto sizeA = polygon.hull.size();
        const auto limitA = sizeA == 2 ? 1 : sizeA;

        // Check if circle and polygon edges intersect.
        for (size_t i = 0; i < limitA; ++i) {
            // Transform vertices to `shB` (cylinder) space. The cylinder axis
            // is the x-axis.
            auto idx0A = polygon.hull[i];
            auto idx1A = polygon.hull[(i + 1) % sizeA];
            auto v0A = polygon.vertices[idx0A];
            auto v1A = polygon.vertices[idx1A];

            auto v0B = to_object_space(v0A, posB, ornB);
            auto v1B = to_object_space(v1A, posB, ornB);

            scalar s[2];
            auto num_points = intersect_line_circle(to_vector2_zy(v0B),
                                                    to_vector2_zy(v1B),
                                                    shB.radius, s[0], s[1]);

            for (size_t j = 0; j < num_points; ++j) {
                auto t = s[j];
                if (t < 0 || t> 1) continue;

                point.pivotA = lerp(v0A, v1A, t);
                point.pivotB = lerp(v0B, v1B, t);
                point.pivotB[cyl_ax_idx] = pivotB_axis;
                result.maybe_add_point(point);

                ++num_edge_intersections;
            }
        }

        if (polygon.hull.size() > 2 && num_vertices_in_face == 0 && num_edge_intersections == 0) {
            // Check if cylinder face center is in polygon.
            if (point_in_polygonal_prism(polygon.vertices, polygon.hull, sep_axis, posB)) {
                auto multipliers = std::array<scalar, 4>{0, 1, 0, -1};

                for(int i = 0; i < 4; ++i) {
                    point.pivotB[cyl_ax_idx] = pivotB_axis;
                    point.pivotB[cyl_ax_idx_ortho0] = shB.radius * multipliers[i];
                    point.pivotB[cyl_ax_idx_ortho1] = shB.radius * multipliers[(i + 1) % 4];
                    point.pivotA = to_world_space(point.pivotB, posB, ornB);
                    point.pivotA = project_plane(point.pivotA, polygon.origin, sep_axis);
                    result.maybe_add_point(point);
                }
            }
        }
        break;
    }
    case cylinder_feature::side_edge: {
        std::array<vector3, 2> edge_vertices;
        edge_vertices[0] = face_center_neg + sep_axis * shB.radius;
        edge_vertices[1] = face_center_pos + sep_axis * shB.radius;

        point.normal_attachment = polygon.hull.size() > 2 ?
            contact_normal_attachment::normal_on_A:
            contact_normal_attachment::none;

        // Check if the vertices of the cylinder edge are inside the polygon.
        if (polygon.hull.size() > 2) {
            for (auto &pointB : edge_vertices) {
                if (point_in_polygonal_prism(polygon.vertices, polygon.hull, sep_axis, pointB)) {
                    point.pivotA = project_plane(pointB, polygon.origin, sep_axis);
                    point.pivotB = to_object_space(pointB, posB, ornB);
                    result.maybe_add_point(point);
                }
            }
        }

        if (result.num_points == 2) {
            // Side edge fully contained in polygon. No need to check for intersections.
            break;
        }

        // Check if the cylinder edge intersects the edges of the polygon.
        if (polygon.hull.size() > 1) {
            // If the feature is a polygon, it will be necessary to wrap around the
            // vertex array. If it is just one edge, then avoid calculating the same
            // segment-segment intersection twice.
            const auto sizeA = polygon.hull.size();
            const auto limitA = sizeA == 2 ? 1 : sizeA;

            // Vertices of cylinder side edge on the contact plane space.
            auto t0B = to_object_space(edge_vertices[0], polygon.origin, polygon.basis);
            auto t1B = to_object_space(edge_vertices[1], polygon.origin, polygon.basis);
            auto v0B = to_vector2_xz(t0B);
            auto v1B = to_vector2_xz(t1B);

            scalar s[2], t[2];

            for (size_t i = 0; i < limitA; ++i) {
                auto idx0A = polygon.hull[i];
                auto idx1A = polygon.hull[(i + 1) % sizeA];
                auto &v0A = polygon.plane_vertices[idx0A];
                auto &v1A = polygon.plane_vertices[idx1A];

                auto num_points = intersect_segments(v0A, v1A, v0B, v1B,
                                                     s[0], t[0], s[1], t[1]);

                for (size_t k = 0; k < num_points; ++k) {
                    point.pivotA = lerp(polygon.vertices[idx0A], polygon.vertices[idx1A], s[k]);
                    auto pivotB_world = lerp(edge_vertices[0], edge_vertices[1], t[k]);
                    point.pivotB = to_object_space(pivotB_world, posB, ornB);
                    result.maybe_add_point(point);
                }
            }
        } else {
            // Polygon vertex against cylinder edge.
            EDYN_ASSERT(polygon.hull.size() == 1);
            point.pivotA = polygon.vertices[polygon.hull[0]];
            auto edge_dir = edge_vertices[1] - edge_vertices[0];
            vector3 pivotB_world; scalar t;
            closest_point_line(edge_vertices[0], edge_dir, point.pivotA, t, pivotB_world);
            point.pivotB = to_object_space(pivotB_world, posB, ornB);
            result.add_point(point);
        }
        break;
    }
    case cylinder_feature::cap_edge: {
        auto supportB = shB.support_point(posB, ornB, sep_axis);
        point.pivotA = supportB + sep_axis * distance;
        point.pivotB = to_object_space(supportB, posB, ornB);
        point.normal_attachment = polygon.hull.size() > 2 ?
            contact_normal_attachment::normal_on_A:
            contact_normal_attachment::none;
        result.add_point(point);
        break;
    }
    }
}

void collide(const cylinder_shape &shA, const polyhedron_shape &shB,
             const collision_context &ctx, collision_result &result) {
    swap_collide(shA, shB, ctx, result);
}

}
