#include "edyn/collision/collide.hpp"
#include "edyn/collision/collision_result.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/util/shape_util.hpp"
#include "edyn/math/vector2_3_util.hpp"
#include "edyn/math/math.hpp"
#include "edyn/math/constants.hpp"

namespace edyn {

void collide(const polyhedron_shape &shA, const box_shape &shB,
             const collision_context &ctx, collision_result &result) {
    // Convex polyhedron against box SAT. All calculations done in the 
    // polyhedron's space.
    const auto posB = to_object_space(ctx.posB, ctx.posA, ctx.ornA);
    const auto ornB = conjugate(ctx.ornA) * ctx.ornB;
    const auto threshold = ctx.threshold;
    const auto &meshA = *shA.mesh;

    const auto box_axes = std::array<vector3, 3>{
        quaternion_x(ornB),
        quaternion_y(ornB),
        quaternion_z(ornB)
    };

    scalar distance = -EDYN_SCALAR_MAX;
    scalar projection_poly = EDYN_SCALAR_MAX;
    scalar projection_box = -EDYN_SCALAR_MAX;
    auto sep_axis = vector3_zero;

    // Face normals of polyhedron.
    for (size_t i = 0; i < meshA.num_faces(); ++i) {
        auto normal_world = -meshA.normals[i]; // Point towards polyhedron.

        auto vertex_idx = meshA.first_vertex_index(i);
        auto &vertexA = meshA.vertices[vertex_idx];
        auto vertex_world = vertexA;

        // Find point on box that's furthest along the opposite direction
        // of the face normal.
        auto supB = shB.support_point(posB, ornB, normal_world);
        auto dist = dot(vertex_world - supB, normal_world);

        if (dist > distance) {
            distance = dist;
            projection_poly = dot(vertex_world, normal_world);
            projection_box = dot(supB, normal_world);
            sep_axis = normal_world;
        }
    }

    // Face normals of box.
    for (size_t i = 0; i < 3; ++i) {
        auto dir = box_axes[i];
        if (dot(posB, dir) > 0) {
            dir = -dir; // Point towards polyhedron.
        }

        // Find point on polyhedron that's furthest along the opposite direction
        // of the box face normal.
        auto projA = -point_cloud_support_projection(meshA.vertices, -dir);
        auto projB = dot(posB, dir) + shB.half_extents[i];
        auto dist = projA - projB;

        if (dist > distance) {
            distance = dist;
            projection_poly = projA;
            projection_box = projB;
            sep_axis = dir;
        }
    }

    // Edge vs edge.
    for (size_t i = 0; i < meshA.num_edges(); ++i) {
        auto [vertexA0, vertexA1] = meshA.get_edge(i);
        auto poly_edge = vertexA1 - vertexA0;

        for (auto &box_edge : box_axes) {
            auto dir = cross(poly_edge, box_edge);
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
                projection_box = projB;
                sep_axis = dir;
            }
        }
    }

    if (distance > threshold) {
        return;
    }

    auto polygon = point_cloud_support_polygon<true>(
        meshA.vertices.begin(), meshA.vertices.end(), vector3_zero, 
        sep_axis, projection_poly, true, support_feature_tolerance);

    auto contact_origin_box = sep_axis * projection_box;
    scalar feature_distanceB;
    box_feature featureB;
    size_t feature_indexB;
    shB.support_feature(posB, ornB, contact_origin_box, sep_axis, 
                        featureB, feature_indexB,
                        feature_distanceB, threshold);

    auto normalB = rotate(conjugate(ornB), sep_axis);

    switch (featureB) {
    case box_feature::face: {
        auto face_vertices = shB.get_face(feature_indexB, posB, ornB);

        // Check if vertices of the polyhedron on the contact plane are inside
        // the box face.
        for (auto idxA : polygon.hull) {
            auto &pointA = polygon.vertices[idxA];

            if (point_in_polygonal_prism(face_vertices, sep_axis, pointA)) {
                auto pivotA = pointA;
                auto pivotB_world = project_plane(pointA, contact_origin_box, sep_axis);
                auto pivotB = to_object_space(pivotB_world, posB, ornB);
                result.maybe_add_point({pivotA, pivotB, normalB, distance});
            }
        }

        // If the vertices of the polyhedron at the contact plane form a polygon,
        // check if the vertices of the box face are inside the polygon.
        if (polygon.hull.size() > 2) {
            for (auto &pointB : face_vertices) {
                if (point_in_polygonal_prism(polygon.vertices, polygon.hull, sep_axis, pointB)) {
                    auto pivotB = to_object_space(pointB, posB, ornB);
                    auto pivotA = project_plane(pointB, polygon.origin, sep_axis);
                    result.maybe_add_point({pivotA, pivotB, normalB, distance});
                }
            }
        }

        if (polygon.hull.size() > 1) {
            // Check if the edges of the box face intersects the edges of the polygon.
            std::array<vector2, 4> plane_vertices_box;
            for (size_t i = 0; i < 4; ++i) {
                auto vertex_tangent = to_object_space(face_vertices[i], contact_origin_box, polygon.basis);
                plane_vertices_box[i] = to_vector2_xz(vertex_tangent);
            }

            // If the feature is a polygon, it will be necessary to wrap around the 
            // vertex array. If it is just one edge, then avoid calculating the same
            // segment-segment intersection twice.
            const auto sizeA = polygon.hull.size();
            const auto limitA = sizeA == 2 ? 1 : sizeA;
            scalar s[2], t[2];

            for (size_t i = 0; i < limitA; ++i) {
                auto idx0A = polygon.hull[i];
                auto idx1A = polygon.hull[(i + 1) % sizeA];
                auto &v0A = polygon.plane_vertices[idx0A];
                auto &v1A = polygon.plane_vertices[idx1A];

                for (size_t j = 0; j < 4; ++j) {
                    auto idx0B = j;
                    auto idx1B = (j + 1) % 4;
                    auto &v0B = plane_vertices_box[idx0B];
                    auto &v1B = plane_vertices_box[idx1B];
                    auto num_points = intersect_segments(v0A, v1A, v0B, v1B, 
                                                        s[0], t[0], s[1], t[1]);

                    for (size_t k = 0; k < num_points; ++k) {
                        auto pivotA = lerp(polygon.vertices[idx0A], polygon.vertices[idx1A], s[k]);
                        auto pivotB_world = lerp(face_vertices[idx0B], face_vertices[idx1B], t[k]);
                        auto pivotB = to_object_space(pivotB_world, posB, ornB);
                        result.maybe_add_point({pivotA, pivotB, normalB, distance});
                    }
                }
            }
        }
        break;
    }
    case box_feature::edge: {
        auto edge_vertices = shB.get_edge(feature_indexB, posB, ornB);

        // Check if the vertices of the box edge are inside the polygon.
        if (polygon.hull.size() > 2) {
            for (auto &pointB : edge_vertices) {
                if (point_in_polygonal_prism(polygon.vertices, polygon.hull, sep_axis, pointB)) {
                    auto pivotA = project_plane(pointB, polygon.origin, sep_axis);
                    auto pivotB = to_object_space(pointB, posB, ornB);
                    result.add_point({pivotA, pivotB, normalB, distance});
                }
            }
        }

        // Check if the box edge intersects the edges of the polygon.
        if (polygon.hull.size() > 1) {
            // If the feature is a polygon, it will be necessary to wrap around the 
            // vertex array. If it is just one edge, then avoid calculating the same
            // segment-segment intersection twice.
            const auto sizeA = polygon.hull.size();
            const auto limitA = sizeA == 2 ? 1 : sizeA;

            // Vertices of box edge on the contact plane space.
            auto t0B = to_object_space(edge_vertices[0], contact_origin_box, polygon.basis);
            auto t1B = to_object_space(edge_vertices[1], contact_origin_box, polygon.basis);
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
                    result.add_point({pivotA, pivotB, normalB, distance});
                }
            }
        } else {
            // Polygon vertex against box edge.
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
    case box_feature::vertex: {
        auto pivotB = shB.get_vertex(feature_indexB);
        auto pivotB_world = to_world_space(pivotB, posB, ornB);
        auto pivotA = pivotB_world + sep_axis * distance;
        result.add_point({pivotA, pivotB, normalB, distance});
    }
    }
}

void collide(const box_shape &shA, const polyhedron_shape &shB, 
             const collision_context &ctx, collision_result &result) {
    swap_collide(shA, shB, ctx, result);
}

}
