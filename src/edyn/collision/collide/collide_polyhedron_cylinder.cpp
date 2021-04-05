#include "edyn/collision/collide.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/math/vector2_3_util.hpp"
#include "edyn/shapes/cylinder_shape.hpp"
#include "edyn/util/shape_util.hpp"
#include "edyn/math/math.hpp"

namespace edyn {

collision_result collide(const polyhedron_shape &shA, const cylinder_shape &shB,
                         const collision_context &ctx) {
    // Convex polyhedron against cylinder SAT.
    // Calculate collision with polyhedron in the origin for better floating point
    // precision. Position of cylinder is modified accordingly.
    const auto posA = vector3_zero;
    const auto &ornA = ctx.ornA;
    const auto posB = ctx.posB - ctx.posA;
    const auto &ornB = ctx.ornB;
    const auto threshold = ctx.threshold;

    // The pre-rotated vertices and normals are used to avoid rotating vertices
    // every time.
    auto &rmeshA = *(*ctx.rmeshA);

    const auto cyl_axis = quaternion_x(ornB);
    const auto face_center_pos = posB + cyl_axis * shB.half_length;
    const auto face_center_neg = posB - cyl_axis * shB.half_length;

    scalar distance = -EDYN_SCALAR_MAX;
    scalar projection_poly = EDYN_SCALAR_MAX;
    scalar projection_cyl = -EDYN_SCALAR_MAX;
    auto sep_axis = vector3_zero;

    // Face normals of polyhedron.
    for (size_t i = 0; i < shA.mesh->num_faces(); ++i) {
        auto normal_world = -rmeshA.normals[i]; // Point towards polyhedron.

        auto vertex_idx = shA.mesh->first_vertex_index(i);
        auto &vertexA = rmeshA.vertices[vertex_idx];
        auto vertex_world = vertexA + posA;

        // Find point on box that's furthest along the opposite direction
        // of the face normal.
        auto supB = shB.support_point(posB, ornB, normal_world);
        auto dist = dot(vertex_world - supB, normal_world);

        if (dist > distance) {
            distance = dist;
            projection_poly = dot(vertex_world, normal_world);
            projection_cyl = dot(supB, normal_world);
            sep_axis = normal_world;
        }
    }

    if (distance > threshold) {
        return {};
    }

    // Find all vertices in the polyhedron that are near the projection boundary.
    scalar tolerance = 0.002;
    std::vector<vector3> vertices_poly;
    // Vertices on the 2D contact plane.
    std::vector<vector2> plane_vertices_poly;
    // Points at the contact plane.
    auto contact_origin_poly = sep_axis * projection_poly;
    auto contact_origin_cyl = sep_axis * projection_cyl;
    // Build a basis tangent to the contact plane so calculations can be done
    // in tangent space.
    vector3 contact_tangent0, contact_tangent1;
    plane_space(sep_axis, contact_tangent0, contact_tangent1);
    auto contact_basis = matrix3x3_columns(contact_tangent0, sep_axis, contact_tangent1);

    for (auto &vertex_world : rmeshA.vertices) {
        if (dot(vertex_world, sep_axis) < projection_poly + tolerance) {
            auto vertex_tangent = to_object_space(vertex_world, contact_origin_poly, contact_basis);
            auto vertex_plane = to_vector2_xz(vertex_tangent);
            plane_vertices_poly.push_back(vertex_plane);
            vertices_poly.push_back(vertex_world);
        }
    }

    EDYN_ASSERT(!vertices_poly.empty() && !plane_vertices_poly.empty());

    auto hull_poly = calculate_convex_hull(plane_vertices_poly, tolerance);

    cylinder_feature featureB;
    size_t feature_indexB;
    vector3 supB;
    scalar projectionB;
    shB.support_feature(posB, ornB, contact_origin_cyl, sep_axis, featureB, feature_indexB, supB, projectionB, threshold);

    auto result = collision_result{};
    auto normalB = rotate(conjugate(ornB), sep_axis);

    switch (featureB) {
    case cylinder_feature::face: {
        size_t num_vertices_in_face = 0;      

        // Check if polygon vertices are inside a cylinder cap face (by checking
        // if its distance from the cylinder axis is smaller than the cylinder radius).
        for (auto idxA : hull_poly) {
            auto &pointA = vertices_poly[idxA];
            vector3 closest; scalar t;
            auto dist_sqr = closest_point_line(posB, cyl_axis, pointA, t, closest);

            if (dist_sqr > shB.radius * shB.radius) continue;

            auto pivotA = to_object_space(pointA, posA, ornA);
            auto pivotB = to_object_space(pointA, posB, ornB);
            pivotB.x = shB.half_length * (feature_indexB == 0 ? 1 : -1);
            result.maybe_add_point({pivotA, pivotB, normalB, distance});

            ++num_vertices_in_face;
        }
    
        // If not all vertices of the polygon are contained in the cylinder
        // cap face, there could be edge intersections or the cylinder cap
        // face could be contained within the polygon.
        size_t num_edge_intersections = 0;

        const auto sizeA = hull_poly.size();
        const auto limitA = sizeA == 2 ? 1 : sizeA;
        scalar s[2], t[2];

        // Check if circle and box edges intersect.
        for (size_t i = 0; i < limitA; ++i) {
            // Transform vertices to `shB` (cylinder) space. The cylinder axis
            // is the x-axis.
            auto idx0A = hull_poly[i];
            auto idx1A = hull_poly[(i + 1) % sizeA];
            auto v0A = vertices_poly[idx0A];
            auto v1A = vertices_poly[idx1A];

            auto v0B = to_object_space(v0A, posB, ornB);
            auto v1B = to_object_space(v1A, posB, ornB);

            scalar s[2];
            auto num_points = intersect_line_circle(to_vector2_zy(v0B), 
                                                    to_vector2_zy(v1B), 
                                                    shB.radius, s[0], s[1]);

            for (size_t j = 0; j < num_points; ++j) {
                s[j] = clamp_unit(s[j]);
                auto pivotB = lerp(v0B, v1B, s[j]);
                pivotB.x = shB.half_length * (feature_indexB == 0 ? 1 : -1);
                auto pivotA_world = lerp(v0A, v1A, s[j]);
                auto pivotA = to_object_space(pivotA_world, posA, ornA);
                result.maybe_add_point({pivotA, pivotB, normalB, distance});

                ++num_edge_intersections;
            }
        }

        if (num_vertices_in_face == 0 && num_edge_intersections == 0) {
            // Check if cylinder face center is in polygon.
            if (point_in_polygonal_prism(vertices_poly, hull_poly, sep_axis, posB)) {
                auto multipliers = std::array<scalar, 4>{0, 1, 0, -1};
                for(int i = 0; i < 4; ++i) {
                    auto pivotB_x = shB.half_length * (feature_indexB == 0 ? 1 : -1);
                    auto pivotB = vector3{pivotB_x, 
                                          shB.radius * multipliers[i], 
                                          shB.radius * multipliers[(i + 1) % 4]};
                    auto pivotA_world = to_world_space(pivotB, posB, ornB);
                    pivotA_world = project_plane(pivotA_world, vertices_poly[0], normalB);
                    auto pivotA = to_object_space(pivotA_world, posA, ornB);
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
        if (hull_poly.size() > 2) {
            for (auto &pointB : edge_vertices) {
                if (point_in_polygonal_prism(vertices_poly, hull_poly, sep_axis, pointB)) {
                    auto pivotB = to_object_space(pointB, posB, ornB);
                    auto pivotA_world = project_plane(pointB, contact_origin_poly, sep_axis);
                    auto pivotA = to_object_space(pivotA_world, posA, ornA);
                    result.add_point({pivotA, pivotB, normalB, distance});
                }
            }
        }

        // Check if the cylinder edge intersects the edges of the polygon.
        if (hull_poly.size() > 1) {
            // If the feature is a polygon, it is will be necessary to wrap around the 
            // vertex array. If it is just one edge, then avoid calculating the same
            // segment-segment intersection twice.
            const auto sizeA = hull_poly.size();
            const auto limitA = sizeA == 2 ? 1 : sizeA;

            // Vertices of box edge on the contact plane space.
            auto t0B = to_object_space(edge_vertices[0], contact_origin_cyl, contact_basis);
            auto t1B = to_object_space(edge_vertices[1], contact_origin_cyl, contact_basis);
            auto v0B = to_vector2_xz(t0B);
            auto v1B = to_vector2_xz(t1B);

            scalar s[2], t[2];

            for (size_t i = 0; i < limitA; ++i) {
                auto idx0A = hull_poly[i];
                auto idx1A = hull_poly[(i + 1) % sizeA];
                auto &v0A = plane_vertices_poly[idx0A];
                auto &v1A = plane_vertices_poly[idx1A];

                auto num_points = intersect_segments(v0A, v1A, v0B, v1B, 
                                                    s[0], t[0], s[1], t[1]);

                for (size_t k = 0; k < num_points; ++k) {
                    auto pivotA_world = lerp(vertices_poly[idx0A], vertices_poly[idx1A], s[k]);
                    auto pivotB_world = lerp(edge_vertices[0], edge_vertices[1], t[k]);
                    auto pivotA = to_object_space(pivotA_world, posA, ornA);
                    auto pivotB = to_object_space(pivotB_world, posB, ornB);
                    result.add_point({pivotA, pivotB, normalB, distance});
                }
            }
        } else {
            // Polygon vertex against cylinder edge.
            EDYN_ASSERT(hull_poly.size() == 1);
            auto &pivotA = vertices_poly[hull_poly[0]];
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
        auto pivotA = to_object_space(supB + sep_axis * distance, posA, ornA);
        result.maybe_add_point({pivotA, pivotB, normalB, distance});
        break;
    }
    }

    return result;
}

collision_result collide(const cylinder_shape &shA, const polyhedron_shape &shB, 
                         const collision_context &ctx) {
    return swap_collide(shA, shB, ctx);
}

}
