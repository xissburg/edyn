#include "edyn/collision/collide.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/math/vector2_3_util.hpp"
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

    const auto cyl_axis = quaternion_x(ornB);
    const auto face_center_pos = posB + cyl_axis * shB.half_length;
    const auto face_center_neg = posB - cyl_axis * shB.half_length;

    scalar distance = -EDYN_SCALAR_MAX;
    scalar projection_poly = EDYN_SCALAR_MAX;
    scalar projection_cyl = -EDYN_SCALAR_MAX;
    auto sep_axis = vector3_zero;

    // Face normals of polyhedron.
    for (size_t i = 0; i < shA.mesh->num_faces(); ++i) {
        auto normal_world = -meshA.normals[i]; // Point towards polyhedron.

        auto vertex_idx = shA.mesh->first_vertex_index(i);
        auto &vertex_world = meshA.vertices[vertex_idx];

        // Find point on box that's furthest along the opposite direction
        // of the face normal.
        auto projA = dot(vertex_world, normal_world);
        auto projB = shB.support_projection(posB, ornB, normal_world);
        auto dist = projA - projB;

        if (dist > distance) {
            distance = dist;
            projection_poly = projA;
            projection_cyl = projB;
            sep_axis = normal_world;
        }
    }

    // Cylinder cap face normals.
    for (size_t i = 0; i < 2; ++i) {
        auto dir = std::array<vector3, 2>{cyl_axis, -cyl_axis}[i];
        auto projA = -point_cloud_support_projection(meshA.vertices, -dir);
        auto projB = dot(posB, dir) + shB.half_length;
        auto dist = projA - projB;
        
        if (dist > distance) {
            distance = dist;
            projection_poly = projA;
            projection_cyl = projB;
            sep_axis = dir;
        }
    }

    // Polyhedron edges vs cylinder side edges.
    for (size_t i = 0; i < shA.mesh->num_edges(); ++i) {
        auto [vertexA0, vertexA1] = shA.mesh->get_edge(i);
        auto poly_edge = vertexA1 - vertexA0;

        auto dir = cross(poly_edge, cyl_axis);
        auto dir_len_sqr = length_sqr(dir);

        if (!(dir_len_sqr > EDYN_EPSILON)) {
            continue;
        }

        dir /= std::sqrt(dir_len_sqr);

        if (dot(posB, dir) > 0) {
            // Make it point towards A.
            dir *= -1;
        }

        auto projA = -point_cloud_support_projection(meshA.vertices, -dir);
        auto projB = shB.support_projection(posB, ornB, dir);
        auto dist = projA - projB;

        if (dist > distance) {
            distance = dist;
            projection_poly = projA;
            projection_cyl = projB;
            sep_axis = dir;
        }
    }

    // Polyhedron vertices vs cylinder side edges.
    for (auto &rvertex : meshA.vertices) {
        vector3 closest; scalar t;
        closest_point_line(face_center_neg, cyl_axis, rvertex, t, closest);
        auto dir = rvertex - closest;
        auto dir_len_sqr = length_sqr(dir);

        if (!(dir_len_sqr > EDYN_EPSILON)) {
            continue;
        }

        dir /= std::sqrt(dir_len_sqr);

        if (dot(posB, dir) > 0) {
            // Make it point towards A.
            dir *= -1;
        }

        auto projA = -point_cloud_support_projection(meshA.vertices, -dir);
        auto projB = shB.support_projection(posB, ornB, dir);
        auto dist = projA - projB;

        if (dist > distance) {
            distance = dist;
            projection_poly = projA;
            projection_cyl = projB;
            sep_axis = dir;
        }
    }

    // Cylinder cap edges vs polyhedron edges.
    for (size_t i = 0; i < shA.mesh->num_edges(); ++i) {
        auto [vertexA0, vertexA1] = shA.mesh->get_edge(i);

        // Find closest point between circle and edge. 
        for (size_t j = 0; j < 2; ++j) {
            auto face_center = j == 0 ? face_center_neg : face_center_pos;
            size_t num_points;
            scalar s0, s1;
            vector3 cc0, cl0, cc1, cl1;
            vector3 dir;
            closest_point_circle_line(face_center, ornB, shB.radius, 
                                      vertexA0, vertexA1, 
                                      num_points, s0, cc0, cl0, s1, cc1, cl1, 
                                      dir, support_feature_tolerance);

            if (!(s0 > 0 && s0 < 1)) continue;

            if (dot(posB, dir) > 0) {
                // Make it point towards A.
                dir *= -1;
            }

            auto projA = -point_cloud_support_projection(meshA.vertices, -dir);
            auto projB = shB.support_projection(posB, ornB, dir);
            auto dist = projA - projB;

            if (dist > distance) {
                distance = dist;
                projection_poly = projA;
                projection_cyl = projB;
                sep_axis = dir;
            }
        }
    }

    if (distance > threshold) {
        return;
    }

    auto polygon = point_cloud_support_polygon(
        meshA.vertices.begin(), meshA.vertices.end(), vector3_zero, 
        sep_axis, projection_poly, true, support_feature_tolerance);

    auto contact_origin_cyl = sep_axis * projection_cyl;
    cylinder_feature featureB;
    size_t feature_indexB;
    vector3 supB;
    scalar projectionB;
    shB.support_feature(posB, ornB, contact_origin_cyl, sep_axis, featureB,
                        feature_indexB, supB, projectionB, support_feature_tolerance);

    auto normalB = rotate(conjugate(ornB), sep_axis);

    switch (featureB) {
    case cylinder_feature::face: {
        size_t num_vertices_in_face = 0;      

        // Check if polygon vertices are inside a cylinder cap face (by checking
        // if its distance from the cylinder axis is smaller than the cylinder radius).
        for (auto idxA : polygon.hull) {
            auto &pointA = polygon.vertices[idxA];
            vector3 closest; scalar t;
            auto dist_sqr = closest_point_line(posB, cyl_axis, pointA, t, closest);

            if (dist_sqr > shB.radius * shB.radius) continue;

            auto pivotA = pointA;
            auto pivotB = to_object_space(pointA, posB, ornB);
            pivotB.x = shB.half_length * to_sign(feature_indexB == 0);
            result.maybe_add_point({pivotA, pivotB, normalB, distance});

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
                if (s[j] < 0 || s[j] > 1) continue;

                auto pivotA = lerp(v0A, v1A, s[j]);
                auto pivotB = lerp(v0B, v1B, s[j]);
                pivotB.x = shB.half_length * to_sign(feature_indexB == 0);
                result.maybe_add_point({pivotA, pivotB, normalB, distance});

                ++num_edge_intersections;
            }
        }

        if (polygon.hull.size() > 2 && num_vertices_in_face == 0 && num_edge_intersections == 0) {
            // Check if cylinder face center is in polygon.
            if (point_in_polygonal_prism(polygon.vertices, polygon.hull, sep_axis, posB)) {
                auto multipliers = std::array<scalar, 4>{0, 1, 0, -1};
                auto pivotB_x = shB.half_length * to_sign(feature_indexB == 0);

                for(int i = 0; i < 4; ++i) {
                    auto pivotB = vector3{pivotB_x, 
                                          shB.radius * multipliers[i], 
                                          shB.radius * multipliers[(i + 1) % 4]};
                    auto pivotA = to_world_space(pivotB, posB, ornB);
                    pivotA = project_plane(pivotA, polygon.origin, normalB);
                    result.maybe_add_point({pivotA, pivotB, normalB, distance});
                }
            }
        }
        break;
    }
    case cylinder_feature::side_edge: {
        std::array<vector3, 2> edge_vertices;
        edge_vertices[0] = face_center_neg + sep_axis * shB.radius;
        edge_vertices[1] = face_center_pos + sep_axis * shB.radius;

        // Check if the vertices of the cylinder edge are inside the polygon.
        if (polygon.hull.size() > 2) {
            for (auto &pointB : edge_vertices) {
                if (point_in_polygonal_prism(polygon.vertices, polygon.hull, sep_axis, pointB)) {
                    auto pivotA = project_plane(pointB, polygon.origin, sep_axis);
                    auto pivotB = to_object_space(pointB, posB, ornB);
                    result.maybe_add_point({pivotA, pivotB, normalB, distance});
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
                    auto pivotA = lerp(polygon.vertices[idx0A], polygon.vertices[idx1A], s[k]);
                    auto pivotB_world = lerp(edge_vertices[0], edge_vertices[1], t[k]);
                    auto pivotB = to_object_space(pivotB_world, posB, ornB);
                    result.maybe_add_point({pivotA, pivotB, normalB, distance});
                }
            }
        } else {
            // Polygon vertex against cylinder edge.
            EDYN_ASSERT(polygon.hull.size() == 1);
            auto &pivotA = polygon.vertices[polygon.hull[0]];
            auto edge_dir = edge_vertices[1] - edge_vertices[0];
            vector3 pivotB_world; scalar t;
            closest_point_line(edge_vertices[0], edge_dir, pivotA, t, pivotB_world);
            auto pivotB = to_object_space(pivotB_world, posB, ornB);
            result.add_point({pivotA, pivotB, normalB, distance});
        }
        break;
    }
    case cylinder_feature::cap_edge: {
        auto pivotB = to_object_space(supB, posB, ornB);
        auto pivotA = supB + sep_axis * distance;
        result.add_point({pivotA, pivotB, normalB, distance});
        break;
    }
    }
}

void collide(const cylinder_shape &shA, const polyhedron_shape &shB, 
             const collision_context &ctx, collision_result &result) {
    swap_collide(shA, shB, ctx, result);
}

}
