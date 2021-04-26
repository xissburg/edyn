#include "edyn/collision/collide.hpp"
#include "edyn/math/vector2_3_util.hpp"
#include "edyn/math/math.hpp"
#include "edyn/shapes/cylinder_shape.hpp"

namespace edyn {

// Separating-axis test. Find axis with greatest distance between projection
// intervals.
// Axes to be tested:
// - Cylinder cap normals. Simply find the triangle vertices that are 
//   further down in both directions.
// - Triangle face normal. The cylinder could be laying sideways onto the
//   triangle face, or it could be erect above the triangle face thus having
//   one of its caps sitting on it, or else the axis projections can be 
//   found via the support points of the caps along the negative triangle
//   normal.
// - Cylinder sidewall faces and the cross product between sidewall edges and
//   triangle edges. The sidewalls are thought to have infinitely thin faces 
//   and edges running straight from one cap to the other. They're handled 
//   together using the cross product between the cylinder axis and each
//   triangle axis as a separating axis.
// - Cylinder face edges against triangle edges. The closest point between
//   the circle and edge segment are calculated. The vector connecting them
//   is taken as the separating axis. The projections must then be calculated
//   using support points. 

struct separating_axis_cyl_tri {
    vector3 dir;
    scalar distance;
    cylinder_feature cyl_feature;
    triangle_feature tri_feature;
    size_t cyl_feature_index; // 0: positive face, 1: negative face.
    size_t tri_feature_index; // Vertex index or edge index.
    vector3 pivotA;
    vector3 pivotB;
};

void collide(const cylinder_shape &cylinder, const triangle_shape &tri,
             const collision_context &ctx, collision_result &result) {
    const auto &posA = ctx.posA;
    const auto &ornA = ctx.ornA;
    const auto threshold = ctx.threshold;
    const auto cylinder_axis = quaternion_x(ornA);
    const vector3 cylinder_vertices[] = {
        posA - cylinder_axis * cylinder.half_length,
        posA + cylinder_axis * cylinder.half_length
    };

    std::vector<separating_axis_cyl_tri> sep_axes;

    // Cylinder cap normal. Test both directions of the cylinder axis to
    // cover both caps.
    {
        auto axis = separating_axis_cyl_tri{};
        axis.cyl_feature = cylinder_feature::face;

        // Triangle is single-sided thus choose the cylinder cap that faces the
        // triangle.
        axis.cyl_feature_index = dot(cylinder_axis, tri.normal) > 0 ? 1 : 0;
        axis.dir = axis.cyl_feature_index == 0 ? -cylinder_axis : cylinder_axis;

        get_triangle_support_feature(tri.vertices, posA, cylinder_axis,
                                     axis.tri_feature, axis.tri_feature_index,
                                     axis.distance, support_feature_tolerance);
    
        axis.distance = -(cylinder.half_length + axis.distance);
        if (!tri.ignore_feature(axis.tri_feature, axis.tri_feature_index, axis.dir)) {
            sep_axes.push_back(axis);
        }
    }

    // Triangle face normal.
    {
        auto axis = separating_axis_cyl_tri{};
        axis.tri_feature = triangle_feature::face;
        axis.dir = tri.normal;

        cylinder.support_feature(posA, ornA, tri.vertices[0], -tri.normal,
                            axis.cyl_feature, axis.cyl_feature_index,
                            axis.pivotA, axis.distance, support_feature_tolerance);

        // Make distance negative when penetrating.
        axis.distance *= -1;
        sep_axes.push_back(axis);
    }

    // Cylinder side wall edges.
    for (size_t i = 0; i < 3; ++i) {
        if (tri.is_concave_edge[i]) {
            continue;
        }

        // Test cylinder axis against triangle edge.
        const auto &v0 = tri.vertices[i];
        const auto &v1 = tri.vertices[(i + 1) % 3];
        scalar s, t;
        vector3 p0, p1;
        closest_point_segment_segment(cylinder_vertices[1], cylinder_vertices[0],
                                      v0, v1, s, t, p0, p1);

        if (s > 0 && s < 1) {
            if (t > 0 && t < 1) {
                // Within segment.
                auto axis = separating_axis_cyl_tri{};
                axis.cyl_feature = cylinder_feature::side_edge;
                axis.dir = cross(tri.edges[i], cylinder_axis);

                if (length_sqr(axis.dir) <= EDYN_EPSILON) {
                    // Parallel. Find a vector that's orthogonal to both
                    // which lies in the same plane.
                    auto plane_normal = cross(tri.edges[i], cylinder_vertices[1] - v0);
                    axis.dir = cross(plane_normal, tri.edges[i]);
                }

                if (dot(tri.normal, axis.dir) < 0) {
                    axis.dir *= -1;
                }

                axis.dir = normalize(axis.dir);

                get_triangle_support_feature(tri.vertices, posA, axis.dir,
                                             axis.tri_feature, axis.tri_feature_index,
                                             axis.distance, support_feature_tolerance);
                axis.distance = -(cylinder.radius + axis.distance);
                if (!tri.ignore_feature(axis.tri_feature, axis.tri_feature_index, axis.dir)) {
                    sep_axes.push_back(axis);
                }
            } else if (t == 0) {
                // If the closest point parameter is zero it means it is the first
                // vertex in the edge. It's unecessary to handle the second vertex
                // because it is the first vertex of the next edge in this loop.

                // Find closest point in cylinder segment to this vertex and use the
                // axis connecting them as the separating axis.
                scalar r;
                vector3 closest;
                auto dist_sqr = closest_point_segment(cylinder_vertices[1], cylinder_vertices[0],
                                                        v0, r, closest);

                // Ignore points at the extremes.
                if (r > 0 && r < 1 && dist_sqr > EDYN_EPSILON) {
                    auto axis = separating_axis_cyl_tri{};
                    axis.cyl_feature = cylinder_feature::side_edge;
                    auto dist = std::sqrt(dist_sqr);
                    axis.dir = (closest - v0) / dist;

                    if (dot(tri.normal, axis.dir) < 0) {
                        axis.dir *= -1;
                    }

                    get_triangle_support_feature(tri.vertices, posA, axis.dir,
                                                 axis.tri_feature, axis.tri_feature_index,
                                                 axis.distance, support_feature_tolerance);
                    axis.distance = -(cylinder.radius + axis.distance);
                    if (!tri.ignore_feature(axis.tri_feature, axis.tri_feature_index, axis.dir)) {
                        sep_axes.push_back(axis);
                    }
                }
            }
        }
    }

    // Cylinder face edges.
    for (size_t i = 0; i < 2; ++i) {
        auto disc_center = i == 0 ? cylinder_vertices[1] : cylinder_vertices[0];

        for (size_t j = 0; j < 3; ++j) {
            if (tri.is_concave_edge[j]) {
                continue;
            }

            auto &v0 = tri.vertices[j];
            auto &v1 = tri.vertices[(j + 1) % 3];

            // Find closest point between circle and triangle edge segment. 
            size_t num_points;
            scalar s0, s1;
            vector3 cc0, cl0, cc1, cl1;
            vector3 normal;
            closest_point_circle_line(disc_center, ornA, cylinder.radius, v0, v1,
                                      num_points, s0, cc0, cl0, s1, cc1, cl1,
                                      normal, support_feature_tolerance);
            
            if (s0 > 0 && s0 < 1) {
                if (dot(tri.normal, normal) < 0) {
                    normal *= -1;
                }

                auto axis = separating_axis_cyl_tri{};

                scalar projA, projB;
                cylinder.support_feature(posA, ornA, v0, -normal,
                                         axis.cyl_feature, axis.cyl_feature_index,
                                         axis.pivotA, projA, support_feature_tolerance);

                // Precalculate the pivot on the triangle, which is the point on
                // the edge [v0,v1] closest to the pivot on the cylinder.
                scalar t;
                closest_point_segment(v0, v1, axis.pivotA, t, axis.pivotB);

                get_triangle_support_feature(tri.vertices, v0, normal,
                                             axis.tri_feature, axis.tri_feature_index,
                                             projB, support_feature_tolerance);

                axis.distance = -(projA + projB);
                axis.dir = normal;
                if (!tri.ignore_feature(axis.tri_feature, axis.tri_feature_index, axis.dir)) {
                    sep_axes.push_back(axis);
                }
            }
        }
    }

    // Get axis with greatest penetration.
    auto penetration = -EDYN_SCALAR_MAX;
    auto sep_axis_idx = SIZE_MAX;

    for (size_t i = 0; i < sep_axes.size(); ++i) {
        auto &axis = sep_axes[i];

        if (axis.distance > penetration) {
            sep_axis_idx = i;
            penetration = axis.distance;
        }
    }

    if (penetration > threshold || sep_axis_idx == SIZE_MAX) {
        return;
    }

    auto &sep_axis = sep_axes[sep_axis_idx];

    switch (sep_axis.cyl_feature) {
    case cylinder_feature::face: {
        // Check if vertices are inside the circular face.
        size_t num_vertices_in_face = 0;
        auto num_vertices_to_check = get_triangle_feature_num_vertices(sep_axis.tri_feature);

        for (size_t i = 0; i < num_vertices_to_check; ++i) {
            auto k = (sep_axis.tri_feature_index + i) % 3;
            
            if (tri.ignore_vertex(k, sep_axis.dir)) {
                continue;
            }

            auto vertex = tri.vertices[k];
            vector3 closest; scalar t;
            auto dir = cylinder_vertices[1] - cylinder_vertices[0];
            auto dist_sqr = closest_point_line(cylinder_vertices[0], dir, vertex, t, closest);

            if (dist_sqr <= cylinder.radius * cylinder.radius) {
                auto vertex_in_A = rotate(conjugate(ornA), vertex - posA);
                auto vertex_proj_in_A = vertex_in_A;
                vertex_proj_in_A.x = cylinder.half_length * to_sign(sep_axis.cyl_feature_index == 0);
                result.maybe_add_point({vertex_proj_in_A, vertex, sep_axis.dir, sep_axis.distance});

                ++num_vertices_in_face;
            }
        }

        size_t num_edge_intersections = 0;
        size_t num_edges_to_check = 0;
        size_t last_edge_index = 0;

        if (sep_axis.tri_feature == triangle_feature::edge && 
            num_vertices_in_face < 2) {
            num_edges_to_check = 1;
        } else if (sep_axis.tri_feature == triangle_feature::face && 
                    num_vertices_in_face < 3) {
            num_edges_to_check = 3;
        }

        // Check if circle and triangle edges intersect.
        for (size_t i = 0; i < num_edges_to_check; ++i) {
            auto k = (sep_axis.tri_feature_index + i) % 3;
            if (tri.ignore_edge(k, sep_axis.dir)) {
                continue;
            }

            // Transform vertices to cylinder space.
            auto v0 = tri.vertices[k];
            auto v0_A = to_object_space(v0, posA, ornA);

            auto v1 = tri.vertices[(k + 1) % 3];
            auto v1_A = to_object_space(v1, posA, ornA);

            scalar s[2];
            auto num_points = intersect_line_circle(to_vector2_zy(v0_A),
                                                    to_vector2_zy(v1_A),
                                                    cylinder.radius, s[0], s[1]);

            if (num_points > 0) {
                ++num_edge_intersections;
                last_edge_index = k;

                for (size_t pt_idx = 0; pt_idx < num_points; ++pt_idx) {
                    auto t = clamp_unit(s[pt_idx]);
                    auto pivotA_x = cylinder.half_length * to_sign(sep_axis.cyl_feature_index == 0);
                    auto pivotA = lerp(v0_A, v1_A, t);
                    pivotA.x = pivotA_x;
                    auto pivotB = lerp(v0, v1, t);
                    result.maybe_add_point({pivotA, pivotB, sep_axis.dir, sep_axis.distance});
                }
            }
        }

        if (sep_axis.tri_feature == triangle_feature::face) {
            // If no vertex is contained in the circular face nor there is
            // any edge intersection, it means the circle could be contained
            // in the triangle.
            if (num_vertices_in_face == 0 && num_edge_intersections == 0) {
                // Check if cylinder center is contained in the triangle prism.
                if (point_in_triangle(tri.vertices, tri.normal, posA)) {
                    auto multipliers = std::array<scalar, 4>{0, 1, 0, -1};
                    for(size_t i = 0; i < 4; ++i) {
                        auto pivotA_x = cylinder.half_length * to_sign(sep_axis.cyl_feature_index == 0);
                        auto pivotA = vector3{pivotA_x,
                                                cylinder.radius * multipliers[i],
                                                cylinder.radius * multipliers[(i + 1) % 4]};
                        auto pivotB = posA + rotate(ornA, pivotA);
                        pivotB = project_plane(pivotB, tri.vertices[0], tri.normal);
                        result.maybe_add_point({pivotA, pivotB, sep_axis.dir, sep_axis.distance});
                    }
                }
            } else if (num_edge_intersections == 1) {
                // If it intersects a single edge, only two contact points have been added,
                // thus add extra points to create a stable base.
                auto tangent = cross(tri.normal, tri.edges[last_edge_index]);
                auto other_vertex_idx = (last_edge_index + 2) % 3;
                if (dot(tangent, tri.vertices[other_vertex_idx] - tri.vertices[last_edge_index]) < 0) {
                    tangent *= -1;
                }

                tangent = normalize(tangent);
                auto disc_center = sep_axis.cyl_feature_index == 0 ? cylinder_vertices[1] : cylinder_vertices[0];
                auto pivotA_in_B = disc_center + tangent * cylinder.radius;
                auto pivotA = to_object_space(pivotA_in_B, posA, ornA);
                auto pivotB = project_plane(pivotA_in_B, tri.vertices[0], tri.normal);
                result.maybe_add_point({pivotA, pivotB, sep_axis.dir, sep_axis.distance});
            }
        }
    }
    break;

    case cylinder_feature::side_edge:
        switch (sep_axis.tri_feature) {
        case triangle_feature::face: {
            // Cylinder is on its side laying on the triangle face.
            // Check if cylinder vertices are inside triangle face.
            for (auto &vertex : cylinder_vertices) {
                if (point_in_triangle(tri.vertices, tri.normal, vertex)) {
                    auto pivotA_world = vertex - tri.normal * cylinder.radius;
                    auto pivotA = to_object_space(pivotA_world, posA, ornA);
                    auto pivotB = project_plane(vertex, tri.vertices[0], tri.normal);
                    auto local_distance = dot(pivotA_world - tri.vertices[0], tri.normal);
                    result.maybe_add_point({pivotA, pivotB, sep_axis.dir, local_distance});
                }
            }

            // Both vertices are inside the triangle. Unnecessary to look for intersections.
            if (result.num_points == 2) {
                return;
            }

            // Check if the cylinder edge intersects the triangle edges.
            auto &tri_origin = tri.vertices[0];
            auto tangent = normalize(tri.vertices[1] - tri.vertices[0]);
            auto bitangent = cross(tri.normal, tangent);
            auto tri_basis = matrix3x3_columns(tangent, tri.normal, bitangent);

            auto p0 = to_vector2_xz(to_object_space(cylinder_vertices[0], tri_origin, tri_basis));
            auto p1 = to_vector2_xz(to_object_space(cylinder_vertices[1], tri_origin, tri_basis));

            for (int i = 0; i < 3; ++i) {
                // Ignore concave edges.
                if (tri.is_concave_edge[i]) {
                    continue;
                }
                
                auto &v0 = tri.vertices[i];
                auto &v1 = tri.vertices[(i + 1) % 3];
                auto q0 = to_vector2_xz(to_object_space(v0, tri_origin, tri_basis));
                auto q1 = to_vector2_xz(to_object_space(v1, tri_origin, tri_basis));

                scalar s[2], t[2];
                auto num_points = intersect_segments(p0, p1, q0, q1, s[0], t[0], s[1], t[1]);

                for (size_t k = 0; k < num_points; ++k) {
                    auto pivotA_world = lerp(cylinder_vertices[0], cylinder_vertices[1], s[k]) - sep_axis.dir * cylinder.radius;
                    auto pivotA = to_object_space(pivotA_world, posA, ornA);
                    auto pivotB = lerp(v0, v1, t[k]);
                    auto local_distance = dot(pivotA_world - tri.vertices[0], sep_axis.dir);
                    result.maybe_add_point({pivotA, pivotB, sep_axis.dir, local_distance});
                }
            }
            break;
        }
        case triangle_feature::edge: {
            // Already checked if this edge should have been ignored.
            auto v0 = tri.vertices[sep_axis.tri_feature_index];
            auto v1 = tri.vertices[(sep_axis.tri_feature_index + 1) % 3];
            scalar s[2], t[2];
            vector3 p0[2], p1[2];
            size_t num_points = 0;
            closest_point_segment_segment(cylinder_vertices[1], cylinder_vertices[0], v0, v1,
                                          s[0], t[0], p0[0], p1[0], &num_points, 
                                          &s[1], &t[1], &p0[1], &p1[1]);

            for (size_t i = 0; i < num_points; ++i) {
                auto pA_in_B = p0[i] - sep_axis.dir * cylinder.radius;
                auto pA = to_object_space(pA_in_B, posA, ornA);
                result.maybe_add_point({pA, p1[i], sep_axis.dir, sep_axis.distance});
            }
            break;
        }
        case triangle_feature::vertex: {
            // Already checked if this vertex should have been ignored.
            auto pivotB = tri.vertices[sep_axis.tri_feature_index];
            vector3 pivotA; scalar t;
            closest_point_segment(cylinder_vertices[0], cylinder_vertices[1], pivotB, t, pivotA);
            pivotA -= sep_axis.dir * cylinder.radius;
            pivotA = to_object_space(pivotA, posA, ornA);
            result.maybe_add_point({pivotA, pivotB, sep_axis.dir, sep_axis.distance});
            break;
        }
        }
    break;

    case cylinder_feature::cap_edge: {
        switch (sep_axis.tri_feature) {
        case triangle_feature::face: {
            if (point_in_triangle(tri.vertices, tri.normal, sep_axis.pivotA)) {
                auto pivotA = to_object_space(sep_axis.pivotA, posA, ornA);
                auto pivotB = project_plane(sep_axis.pivotA, tri.vertices[0], tri.normal);
                result.maybe_add_point({pivotA, pivotB, sep_axis.dir, sep_axis.distance});
            }
            break;
        }
        case triangle_feature::edge: {
            auto pivotA = to_object_space(sep_axis.pivotA, posA, ornA);
            result.maybe_add_point({pivotA, sep_axis.pivotB, sep_axis.dir, sep_axis.distance});
            break;
        }
        case triangle_feature::vertex: {
            break;
        }
        }
        break;
    }
    }
}

}
