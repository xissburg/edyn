#include "edyn/collision/collide.hpp"
#include "edyn/shapes/triangle_shape.hpp"
#include "edyn/math/math.hpp"
#include <algorithm>

namespace edyn {

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

void collide_cylinder_triangle(
    const cylinder_shape &cylinder, const vector3 &posA, const quaternion &ornA,
    const vector3 &disc_center_pos, const vector3 &disc_center_neg,
    const vector3 &cylinder_axis, const triangle_vertices &vertices, 
    const std::array<bool, 3> &is_concave_edge, const std::array<scalar, 3> &cos_angles,
    scalar threshold, collision_result &result) {

    std::vector<separating_axis_cyl_tri> sep_axes;

    const auto edges = get_triangle_edges(vertices);
    const auto tri_normal = normalize(cross(edges[0], edges[1]));
    std::array<bool, 3> is_concave_vertex;

    for (int i = 0; i < 3; ++i) {
        // If edge starting or ending in this vertex are concave, thus is the vertex.
        is_concave_vertex[i] = is_concave_edge[i] || is_concave_edge[(i + 2) % 3];
    }

    vector3 edge_tangents[3];

    for (int i = 0; i < 3; ++i) {
        edge_tangents[i] = cross(edges[i], tri_normal);
    }

    auto ignore_edge = [&](size_t idx, const vector3 &dir) {
        return is_concave_edge[idx] ||
               dot(dir, edge_tangents[idx]) < -EDYN_EPSILON ||
               dot(dir, tri_normal) < cos_angles[idx];
    };

    auto ignore_vertex = [&](size_t idx, const vector3 &dir) {
        if (is_concave_vertex[idx]) {
            return true;
        }

        auto dot_tangent_0 = dot(dir, edge_tangents[idx]);
        auto dot_tangent_1 = dot(dir, edge_tangents[(idx + 2) % 3]);

        if (dot_tangent_0 < -EDYN_EPSILON && dot_tangent_1 < -EDYN_EPSILON) {
            return true;
        }

        return dot(dir, tri_normal) < cos_angles[idx];
    };

    auto ignore_triangle_feature = [&](triangle_feature tri_feature, size_t idx, const vector3 &dir) {
        switch (tri_feature) {
        case TRIANGLE_FEATURE_EDGE:
            return ignore_edge(idx, dir); break;
        case TRIANGLE_FEATURE_VERTEX:
            return ignore_vertex(idx, dir); break;
        case TRIANGLE_FEATURE_FACE:
            return false;
        }
    };

    // Cylinder cap normal. Test both directions of the cylinder axis to
    // cover both caps.
    {
        auto axis = separating_axis_cyl_tri{};
        axis.cyl_feature = CYLINDER_FEATURE_FACE;

        // Triangle is single-sided thus choose the cylinder cap that faces the
        // triangle.
        axis.cyl_feature_index = dot(cylinder_axis, tri_normal) > 0 ? 1 : 0;
        axis.dir = axis.cyl_feature_index == 0 ? -cylinder_axis : cylinder_axis;

        get_triangle_support_feature(vertices, posA, cylinder_axis, 
                                     axis.tri_feature, axis.tri_feature_index, 
                                     axis.distance, threshold);
    
        axis.distance = -(cylinder.half_length + axis.distance);
        if (!ignore_triangle_feature(axis.tri_feature, axis.tri_feature_index, axis.dir)) {
            sep_axes.push_back(axis);
        }
    }

    // Triangle face normal.
    {
        auto axis = separating_axis_cyl_tri{};
        axis.tri_feature = TRIANGLE_FEATURE_FACE;
        axis.dir = tri_normal;

        cylinder.support_feature(posA, ornA, vertices[0], -tri_normal, 
                            axis.cyl_feature, axis.cyl_feature_index, 
                            axis.pivotA, axis.distance, threshold);

        // Make distance negative when penetrating.
        axis.distance *= -1;
        sep_axes.push_back(axis);
    }

    // Cylinder side wall edges.
    for (size_t i = 0; i < 3; ++i) {
        if (is_concave_edge[i]) {
            continue;
        }

        // Test cylinder axis against triangle edge.
        const auto &v0 = vertices[i];
        const auto &v1 = vertices[(i + 1) % 3];
        scalar s, t;
        vector3 p0, p1;
        closest_point_segment_segment(disc_center_pos, disc_center_neg, 
                                      v0, v1, s, t, p0, p1);

        if (s > 0 && s < 1) {
            if (t > 0 && t < 1) {
                // Within segment.
                auto axis = separating_axis_cyl_tri{};
                axis.cyl_feature = CYLINDER_FEATURE_SIDE_EDGE;
                axis.dir = cross(edges[i], cylinder_axis);

                if (length_sqr(axis.dir) <= EDYN_EPSILON) {
                    // Parallel. Find a vector that's orthogonal to both
                    // which lies in the same plane.
                    auto plane_normal = cross(edges[i], disc_center_pos - v0);
                    axis.dir = cross(plane_normal, edges[i]);
                }

                if (dot(tri_normal, axis.dir) < 0) {
                    axis.dir *= -1;
                }

                axis.dir = normalize(axis.dir);

                get_triangle_support_feature(vertices, posA, axis.dir, 
                                             axis.tri_feature, axis.tri_feature_index, 
                                             axis.distance, threshold);
                axis.distance = -(cylinder.radius + axis.distance);
                if (!ignore_triangle_feature(axis.tri_feature, axis.tri_feature_index, axis.dir)) {
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
                auto dist_sqr = closest_point_segment(disc_center_pos, disc_center_neg, 
                                                        v0, r, closest);

                // Ignore points at the extremes.
                if (r > 0 && r < 1 && dist_sqr > EDYN_EPSILON) {
                    auto axis = separating_axis_cyl_tri{};
                    axis.cyl_feature = CYLINDER_FEATURE_SIDE_EDGE;
                    auto dist = std::sqrt(dist_sqr);
                    axis.dir = (closest - v0) / dist;

                    if (dot(tri_normal, axis.dir) < 0) {
                        axis.dir *= -1;
                    }

                    get_triangle_support_feature(vertices, posA, axis.dir, 
                                            axis.tri_feature, axis.tri_feature_index, 
                                            axis.distance, threshold);
                    axis.distance = -(cylinder.radius + axis.distance);
                    if (!ignore_triangle_feature(axis.tri_feature, axis.tri_feature_index, axis.dir)) {
                        sep_axes.push_back(axis);
                    }
                }
            }
        }
    }

    // Cylinder face edges.
    for (size_t i = 0; i < 2; ++i) {
        auto disc_center = i == 0 ? disc_center_pos : disc_center_neg;

        for (size_t j = 0; j < 3; ++j) {
            if (is_concave_edge[j]) {
                continue;
            }

            auto &v0 = vertices[j];
            auto &v1 = vertices[(j + 1) % 3];

            // Find closest point between circle and triangle edge segment. 
            size_t num_points;
            scalar s0, s1;
            vector3 cc0, cl0, cc1, cl1;
            vector3 normal;
            closest_point_circle_line(disc_center, ornA, cylinder.radius, v0, v1, 
                                      num_points, s0, cc0, cl0, s1, cc1, cl1, normal, threshold);
            
            if (s0 > 0 && s0 < 1) {
                if (dot(tri_normal, normal) < 0) {
                    normal *= -1;
                }

                auto axis = separating_axis_cyl_tri{};

                scalar projA, projB;
                cylinder.support_feature(posA, ornA, v0, -normal, 
                                         axis.cyl_feature, axis.cyl_feature_index, 
                                         axis.pivotA, projA, threshold);

                scalar t;
                closest_point_segment(v0, v1, axis.pivotA, t, axis.pivotB);

                get_triangle_support_feature(vertices, v0, normal, axis.tri_feature, 
                                             axis.tri_feature_index, projB, threshold);

                axis.distance = -(projA + projB);
                axis.dir = normal;
                if (!ignore_triangle_feature(axis.tri_feature, axis.tri_feature_index, axis.dir)) {
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
    case CYLINDER_FEATURE_FACE: {
        // Check if vertices are inside the circular face.
        size_t num_vertices_in_face = 0;
        auto num_vertices_to_check = get_triangle_feature_num_vertices(sep_axis.tri_feature);

        for (size_t i = 0; i < num_vertices_to_check; ++i) {
            auto k = (sep_axis.tri_feature_index + i) % 3;
            
            if (ignore_vertex(k, sep_axis.dir)) {
                continue;
            }

            auto vertex = vertices[k];
            vector3 closest; scalar t;
            auto dir = disc_center_pos - disc_center_neg;
            auto dist_sqr = closest_point_line(disc_center_neg, dir, vertex, t, closest);

            if (dist_sqr <= cylinder.radius * cylinder.radius) {
                auto vertex_in_A = rotate(conjugate(ornA), vertex - posA);
                auto vertex_proj_in_A = vertex_in_A;
                vertex_proj_in_A.x = cylinder.half_length * (sep_axis.cyl_feature_index == 0 ? 1 : -1);
                result.maybe_add_point({vertex_proj_in_A, vertex, sep_axis.dir, sep_axis.distance});

                ++num_vertices_in_face;
            }
        }

        size_t num_edge_intersections = 0;
        size_t num_edges_to_check = 0;
        size_t last_edge_index = 0;

        if (sep_axis.tri_feature == TRIANGLE_FEATURE_EDGE && 
            num_vertices_in_face < 2) {
            num_edges_to_check = 1;
        } else if (sep_axis.tri_feature == TRIANGLE_FEATURE_FACE && 
                    num_vertices_in_face < 3) {
            num_edges_to_check = 3;
        }

        // Check if circle and triangle edges intersect.
        for(size_t i = 0; i < num_edges_to_check; ++i) {
            auto k = (sep_axis.tri_feature_index + i) % 3;
            if (ignore_edge(k, sep_axis.dir)) {
                continue;
            }

            // Transform vertices to cylinder space.
            auto v0 = vertices[k];
            auto v0_A = to_object_space(v0, posA, ornA);

            auto v1 = vertices[(k + 1) % 3];
            auto v1_A = to_object_space(v1, posA, ornA);

            scalar s0, s1;
            auto num_points = intersect_line_circle(v0_A.z, v0_A.y, 
                                                    v1_A.z, v1_A.y, 
                                                    cylinder.radius, s0, s1);

            if(num_points > 0) {
                ++num_edge_intersections;
                last_edge_index = k;

                s0 = clamp_unit(s0);
                auto pivotA_x = cylinder.half_length * (sep_axis.cyl_feature_index == 0 ? 1 : -1);
                auto pivotA = lerp(v0_A, v1_A, s0);
                pivotA.x = pivotA_x;
                auto pivotB = lerp(v0, v1, s0);
                result.maybe_add_point({pivotA, pivotB, sep_axis.dir, sep_axis.distance});

                if (num_points == 2) {
                    s1 = clamp_unit(s1);
                    auto pivotA = lerp(v0_A, v1_A, s1);
                    pivotA.x = pivotA_x;
                    auto pivotB = lerp(v0, v1, s1);
                    result.maybe_add_point({pivotA, pivotB, sep_axis.dir, sep_axis.distance});
                }
            }
        }

        if (sep_axis.tri_feature == TRIANGLE_FEATURE_FACE) {
            // If no vertex is contained in the circular face nor there is
            // any edge intersection, it means the circle could be contained
            // in the triangle.
            if (num_vertices_in_face == 0 && num_edge_intersections == 0) {
                // Check if cylinder center is contained in the triangle prism.
                if (point_in_triangle(vertices, tri_normal, posA)) {
                    auto multipliers = std::array<scalar, 4>{0, 1, 0, -1};
                    for(size_t i = 0; i < 4; ++i) {
                        auto pivotA_x = cylinder.half_length * (sep_axis.cyl_feature_index == 0 ? 1 : -1);
                        auto pivotA = vector3{pivotA_x, 
                                                cylinder.radius * multipliers[i], 
                                                cylinder.radius * multipliers[(i + 1) % 4]};
                        auto pivotB = posA + rotate(ornA, pivotA);
                        pivotB = project_plane(pivotB, vertices[0], tri_normal);
                        result.maybe_add_point({pivotA, pivotB, sep_axis.dir, sep_axis.distance});
                    }
                }
            } else if (num_edge_intersections == 1) {
                // If it intersects a single edge, only two contact points have been added,
                // thus add extra points to create a stable base.
                auto tangent = cross(tri_normal, edges[last_edge_index]);
                auto other_vertex_idx = (last_edge_index + 2) % 3;
                if (dot(tangent, vertices[other_vertex_idx] - vertices[last_edge_index]) < 0) {
                    tangent *= -1;
                }

                tangent = normalize(tangent);
                auto disc_center = sep_axis.cyl_feature_index == 0 ? disc_center_pos : disc_center_neg;
                auto pivotA_in_B = disc_center + tangent * cylinder.radius;
                auto pivotA = to_object_space(pivotA_in_B, posA, ornA);
                auto pivotB = project_plane(pivotA_in_B, vertices[0], tri_normal);
                result.maybe_add_point({pivotA, pivotB, sep_axis.dir, sep_axis.distance});
            }
        }
    }
    break;

    case CYLINDER_FEATURE_SIDE_EDGE:
        switch (sep_axis.tri_feature) {
        case TRIANGLE_FEATURE_FACE: {
            // Cylinder is on its side laying on the triangle face.
            // Segment-triangle intersection/containment test.
            auto c0_in_tri = point_in_triangle(vertices, tri_normal, disc_center_pos);
            auto c1_in_tri = point_in_triangle(vertices, tri_normal, disc_center_neg);

            if (c0_in_tri) {
                auto p0 = disc_center_pos - tri_normal * cylinder.radius;
                auto p0_A = to_object_space(p0, posA, ornA);
                auto pivotB = project_plane(disc_center_pos, vertices[0], tri_normal);
                result.maybe_add_point({p0_A, pivotB, sep_axis.dir, sep_axis.distance});
            }

            if (c1_in_tri) {
                auto p1 = disc_center_neg - tri_normal * cylinder.radius;
                auto p1_A = to_object_space(p1, posA, ornA);
                auto pivotB = project_plane(disc_center_neg, vertices[0], tri_normal);
                result.maybe_add_point({p1_A, pivotB, sep_axis.dir, sep_axis.distance});
            }

            if (!c0_in_tri || !c1_in_tri) {
                // One of them is outside. Find closest points in segments.
                for (size_t i = 0; i < 3; ++i) {
                    // Ignore concave edges.
                    if (is_concave_edge[i]) {
                        continue;
                    }

                    scalar s[2], t[2];
                    vector3 p0[2], p1[2];
                    size_t num_points = 0;
                    closest_point_segment_segment(disc_center_pos, disc_center_neg, 
                                                    vertices[i], vertices[(i + 1) % 3],
                                                    s[0], t[0], p0[0], p1[0], &num_points, 
                                                    &s[1], &t[1], &p0[1], &p1[1]);

                    for (size_t i = 0; i < num_points; ++i) {
                        if (s[i] > 0 && s[i] < 1 && t[i] > 0 && t[i] < 1) {
                            auto pA_in_B = p0[i] - tri_normal * cylinder.radius;
                            auto pA = to_object_space(pA_in_B, posA, ornA);
                            result.maybe_add_point({pA, p1[i], sep_axis.dir, sep_axis.distance});
                        }
                    }
                }
            }
            
            break;
        }
        case TRIANGLE_FEATURE_EDGE: {
            // Already checked if this edge should have been ignored.
            auto v0 = vertices[sep_axis.tri_feature_index];
            auto v1 = vertices[(sep_axis.tri_feature_index + 1) % 3];
            scalar s[2], t[2];
            vector3 p0[2], p1[2];
            size_t num_points = 0;
            closest_point_segment_segment(disc_center_pos, disc_center_neg, v0, v1,
                                            s[0], t[0], p0[0], p1[0], &num_points, 
                                            &s[1], &t[1], &p0[1], &p1[1]);

            for (size_t i = 0; i < num_points; ++i) {
                auto pA_in_B = p0[i] - sep_axis.dir * cylinder.radius;
                auto pA = to_object_space(pA_in_B, posA, ornA);
                result.maybe_add_point({pA, p1[i], sep_axis.dir, sep_axis.distance});
            }
            break;
        }
        case TRIANGLE_FEATURE_VERTEX: {
            // Already checked if this vertex should have been ignored.
            auto pivotB = vertices[sep_axis.tri_feature_index];
            vector3 pivotA; scalar t;
            closest_point_segment(disc_center_neg, disc_center_pos, pivotB, t, pivotA);
            pivotA -= sep_axis.dir * cylinder.radius;
            pivotA = to_object_space(pivotA, posA, ornA);
            result.maybe_add_point({pivotA, pivotB, sep_axis.dir, sep_axis.distance});
            break;
        }
        }
    break;

    case CYLINDER_FEATURE_FACE_EDGE: {
        switch (sep_axis.tri_feature) {
        case TRIANGLE_FEATURE_FACE: {
            if (point_in_triangle(vertices, tri_normal, sep_axis.pivotA)) {
                auto pivotA = to_object_space(sep_axis.pivotA, posA, ornA);
                auto pivotB = project_plane(sep_axis.pivotA, vertices[0], tri_normal);
                result.maybe_add_point({pivotA, pivotB, sep_axis.dir, sep_axis.distance});
            }
            break;
        }
        case TRIANGLE_FEATURE_EDGE: {
            auto pivotA = to_object_space(sep_axis.pivotA, posA, ornA);
            result.maybe_add_point({pivotA, sep_axis.pivotB, sep_axis.dir, sep_axis.distance});
            break;
        }
        case TRIANGLE_FEATURE_VERTEX: {
            break;
        }
        }
        break;
    }
    }
}

}