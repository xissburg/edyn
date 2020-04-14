#include "edyn/collision/collide.hpp"
#include "edyn/math/math.hpp"
#include <algorithm>

namespace edyn {

enum cylinder_feature {
    CYLINDER_FEATURE_FACE,
    CYLINDER_FEATURE_SIDE_EDGE,
    CYLINDER_FEATURE_CIRCLE_EDGE,
};

enum triangle_feature {
    TRIANGLE_FEATURE_VERTEX,
    TRIANGLE_FEATURE_EDGE,
    TRIANGLE_FEATURE_FACE
};

struct separating_axis {
    vector3 pos;
    vector3 dir;
    scalar distance;
    cylinder_feature cyl_feature;
    triangle_feature tri_feature;
    uint16_t cyl_feature_index; // 0: positive face, 1: negative face.
    uint16_t tri_feature_index;
    vector3 pivotA;
};

collision_result collide(const cylinder_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const mesh_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold) {
    auto result = collision_result{};

    // Cylinder position in mesh's space.
    auto posA_in_B = rotate(conjugate(ornB), posA - posB);
    auto ornA_in_B = conjugate(ornB) * ornA;
    auto aabb = shA.aabb(posA_in_B, ornA_in_B);
    shB.trimesh->visit(aabb, [&] (size_t tri_idx, const triangle_vertices &vertices) {
        if (result.num_points == max_contacts) {
            return;
        }

        std::vector<separating_axis> sep_axes;

        auto edges = get_triangle_edges(vertices);
        auto tri_normal = normalize(cross(edges[0], edges[1]));

        // Cylinder cap normal.
        {
            auto axis = separating_axis{};
            axis.dir = quaternion_x(ornA_in_B);
            axis.pos = posA_in_B;
            
            auto minA = -shA.half_length;
            auto maxA = shA.half_length;
            auto minB = large_scalar;
            auto maxB = -large_scalar;

            triangle_feature min_tri_feature, max_tri_feature;
            uint16_t min_tri_feature_index, max_tri_feature_index;

            for (uint16_t i = 0; i < 3; ++i) {
                auto &v = vertices[i];
                auto proj = dot(v - axis.pos, axis.dir);

                if (std::abs(proj - maxB) < EDYN_EPSILON) {
                    if (max_tri_feature == TRIANGLE_FEATURE_VERTEX) {
                        max_tri_feature = TRIANGLE_FEATURE_EDGE;
                        if (i == 2) {
                            // If this is the third vertex (index 2), the previous could
                            // have been either vertex 1 or 0. If 0, then the edge is the
                            // last one, i.e. edge 2. If 1, then the edge is #1.
                            if (max_tri_feature_index == 0) {
                                max_tri_feature_index = 2;
                            } else {
                                max_tri_feature_index = 1;
                            }
                        } else {
                            // If this is the second vertex (index 1), the previous could
                            // only have been vertex 0, thus this must be edge 0.
                            max_tri_feature_index = 0;
                        }
                    } else if (max_tri_feature == TRIANGLE_FEATURE_EDGE) {
                        max_tri_feature = TRIANGLE_FEATURE_FACE;
                    }
                } else if (proj > maxB) {
                    maxB = proj;
                    max_tri_feature = TRIANGLE_FEATURE_VERTEX;
                    max_tri_feature_index = i;
                }
                
                if (std::abs(proj - minB) < EDYN_EPSILON) {
                    if (min_tri_feature == TRIANGLE_FEATURE_VERTEX) {
                        min_tri_feature = TRIANGLE_FEATURE_EDGE;
                        if (i == 2) {
                            // If this is the third vertex (index 2), the previous could
                            // have been either vertex 1 or 0. If 0, then the edge is the
                            // last one, i.e. edge 2. If 1, then the edge is #1.
                            if (min_tri_feature_index == 0) {
                                min_tri_feature_index = 2;
                            } else {
                                min_tri_feature_index = 1;
                            }
                        } else {
                            // If this is the second vertex (index 1), the previous could
                            // only have been vertex 0, thus this must be edge 0.
                            min_tri_feature_index = 0;
                        }
                    } else if (max_tri_feature == TRIANGLE_FEATURE_EDGE) {
                        min_tri_feature = TRIANGLE_FEATURE_FACE;
                    }
                } else if (proj < minB) {
                    minB = proj;
                    min_tri_feature = TRIANGLE_FEATURE_VERTEX;
                    min_tri_feature_index = i;
                }
            }

            axis.cyl_feature = CYLINDER_FEATURE_FACE;

            if (minB > maxA) {
                // B is after A (i.e. wrt `axis.dir`).
                axis.distance = minB - maxA;
                // Positive cylinder face.
                axis.cyl_feature_index = 0;
                // Triangle features at the opposite direction of the separating axis.
                axis.tri_feature = min_tri_feature;
                axis.tri_feature_index = min_tri_feature_index;
                // Make direction point from B to A so it can be correctly used as contact normal.
                axis.dir *= -1;
            } else if (maxB < minA) {
                // B is before A.
                axis.distance = minA - maxB;
                axis.cyl_feature_index = 1;
                axis.tri_feature = max_tri_feature;
                axis.tri_feature_index = max_tri_feature_index;
            } else {
                // Ranges intersect.
                if (minB < minA) {
                    // Left side of B is before left side of A.
                    if (maxB > maxA) {
                        // A's projection is contained in B's.
                        if (maxA - minB < maxB - minA) {
                            axis.cyl_feature_index = 0;
                            axis.tri_feature = min_tri_feature;
                            axis.tri_feature_index = min_tri_feature_index;
                            axis.distance = minB - maxA;
                            axis.dir *= -1;
                        } else {
                            axis.cyl_feature_index = 1;
                            axis.tri_feature = max_tri_feature;
                            axis.tri_feature_index = max_tri_feature_index;
                            axis.distance = minA - maxB;
                        }
                    } else {
                        // Right side of B is before right side of A.
                        axis.distance = minA - maxB;
                        axis.cyl_feature_index = 1;
                        axis.tri_feature = max_tri_feature;
                        axis.tri_feature_index = max_tri_feature_index;
                    }
                } else {
                    if (maxB < maxA) {
                        // B's projection is contained in A's.
                        if (maxA - minB < maxB - minA) {
                            axis.cyl_feature_index = 0;
                            axis.tri_feature = min_tri_feature;
                            axis.tri_feature_index = min_tri_feature_index;
                            axis.distance = minB - maxA;
                            axis.dir *= -1;
                        } else {
                            axis.cyl_feature_index = 1;
                            axis.tri_feature = max_tri_feature;
                            axis.tri_feature_index = max_tri_feature_index;
                            axis.distance = minA - maxB;
                        }
                    } else {
                        axis.distance = minB - maxA;
                        axis.cyl_feature_index = 0;
                        axis.tri_feature = min_tri_feature;
                        axis.tri_feature_index = min_tri_feature_index;
                    }
                }
            }

            if (axis.distance < threshold) {
                sep_axes.push_back(axis);
            }
        }

        // Face normal.
        {
            auto axis = separating_axis{};
            axis.tri_feature = TRIANGLE_FEATURE_FACE;

            axis.dir = tri_normal;
            axis.pos = vertices[0];

            auto cyl_axis = quaternion_x(ornA_in_B);
            auto axis_dot = dot(tri_normal, cyl_axis);

            if (std::abs(axis_dot) + EDYN_EPSILON >= 1) {
                axis.cyl_feature = CYLINDER_FEATURE_FACE;
                axis.cyl_feature_index = axis_dot > 0 ? 1 : 0;
                auto disc_center = posA_in_B + cyl_axis * shA.half_length * (axis_dot > 0 ? -1 : 1);
                axis.distance = dot(tri_normal, disc_center - vertices[0]);
            } else {
                auto disc_center_pos = posA_in_B + cyl_axis * shA.half_length;
                auto disc_center_neg = posA_in_B - cyl_axis * shA.half_length;
                auto sup_pos = support_point_circle(shA.radius, disc_center_pos, ornA_in_B, -tri_normal);
                auto sup_neg = support_point_circle(shA.radius, disc_center_neg, ornA_in_B, -tri_normal);
                auto proj_pos = dot(sup_pos - axis.pos, axis.dir);
                auto proj_neg = dot(sup_neg - axis.pos, axis.dir);

                if (std::abs(proj_pos - proj_neg) < EDYN_EPSILON) {
                    // Cylinder side is parallel to triangle face.
                    axis.cyl_feature = CYLINDER_FEATURE_SIDE_EDGE;
                    axis.distance = proj_pos;
                } else {
                    axis.cyl_feature = CYLINDER_FEATURE_CIRCLE_EDGE;

                    // Select deepest circle.
                    if (proj_pos < proj_neg) {
                        axis.cyl_feature_index = 0;
                        axis.distance = proj_pos;
                        axis.pivotA = sup_pos;
                    } else {
                        axis.cyl_feature_index = 1;
                        axis.distance = proj_neg;
                        axis.pivotA = sup_neg;
                    }
                }
            }

            if (axis.distance < threshold) {
                sep_axes.push_back(axis);
            }
        }

        // Get axis with greatest penetration.
        auto penetration = -EDYN_SCALAR_MAX;
        uint16_t sep_axis_idx = UINT16_MAX;

        for (size_t i = 0; i < sep_axes.size(); ++i) {
            auto &axis = sep_axes[i];

            if (axis.distance > penetration) {
                sep_axis_idx = i;
                penetration = axis.distance;
            }
        }

        if (sep_axis_idx != UINT16_MAX) {
            auto &axis = sep_axes[sep_axis_idx];

            switch (axis.cyl_feature) {
            case CYLINDER_FEATURE_FACE:
                switch (axis.tri_feature) {
                case TRIANGLE_FEATURE_VERTEX: {
                    auto vertex = vertices[axis.tri_feature_index];
                    auto vertex_world = posB + rotate(ornB, vertex);
                    auto vertex_in_A = rotate(conjugate(ornA), vertex_world - posA);
                    auto vertex_proj_in_A = vertex_in_A;
                    vertex_proj_in_A.x = shA.half_length * (axis.tri_feature_index == 0 ? 1 : -1);

                    auto idx = result.num_points++;
                    result.point[idx].pivotA = vertex_proj_in_A;
                    result.point[idx].pivotB = vertex;
                    result.point[idx].normalB = axis.dir;
                    result.point[idx].distance = penetration;
                    break;
                }

                case TRIANGLE_FEATURE_EDGE: {
                    auto v0 = vertices[axis.tri_feature_index];
                    auto v0_world = posB + rotate(ornB, v0);
                    auto v0_A = rotate(conjugate(ornA), v0_world - posA);

                    auto v1 = vertices[(axis.tri_feature_index + 1) % 3];
                    auto v1_world = posB + rotate(ornB, v1);
                    auto v1_A = rotate(conjugate(ornA), v1_world - posA);

                    scalar s0, s1;
                    auto num_points = intersect_line_circle(v0_A.z, v0_A.y, 
                                                            v1_A.z, v1_A.y, 
                                                            shA.radius, s0, s1);

                    EDYN_ASSERT(num_points > 0);

                    s0 = clamp_unit(s0);
                    auto p0 = lerp(v0_A, v1_A, s0);
                    auto pivotA_x = shA.half_length * (axis.cyl_feature_index == 0 ? 1 : -1);
                    auto p0_world = posA + rotate(ornA, p0);
                    auto p0_B = rotate(conjugate(ornB), p0_world - posB);

                    auto idx = result.num_points++;
                    result.point[idx].pivotA = {pivotA_x, p0.y, p0.z};
                    result.point[idx].pivotB = p0_B;
                    result.point[idx].normalB = axis.dir;
                    result.point[idx].distance = penetration;

                    if (num_points == 2) {
                        s1 = clamp_unit(s1);
                        auto p1 = lerp(v0_A, v1_A, s1);
                        auto p1_world = posA + rotate(ornA, p1);
                        auto p1_B = rotate(conjugate(ornB), p1_world - posB);

                        auto idx = result.num_points++;
                        result.point[idx].pivotA = {pivotA_x, p1.y, p1.z};
                        result.point[idx].pivotB = p1_B;
                        result.point[idx].normalB = axis.dir;
                        result.point[idx].distance = penetration;
                    }
                    break;
                }

                case TRIANGLE_FEATURE_FACE: {
                    uint8_t num_edge_intersections = 0;
                    uint8_t last_edge_index;

                    for(uint8_t i = 0; i < 3; ++i) {
                        auto v0 = vertices[i];
                        auto v0_world = posB + rotate(ornB, v0);
                        auto v0_A = rotate(conjugate(ornA), v0_world - posA);

                        auto v1 = vertices[(i + 1) % 3];
                        auto v1_world = posB + rotate(ornB, v1);
                        auto v1_A = rotate(conjugate(ornA), v1_world - posA);

                        scalar s0, s1;
                        auto num_points = intersect_line_circle(v0_A.z, v0_A.y, 
                                                                v1_A.z, v1_A.y, 
                                                                shA.radius, s0, s1);

                        if (num_points == 0) {
                            continue;
                        }

                        ++num_edge_intersections;
                        last_edge_index = i;

                        s0 = clamp_unit(s0);
                        auto p0 = lerp(v0_A, v1_A, s0);
                        auto pivotA_x = shA.half_length * (axis.cyl_feature_index == 0 ? 1 : -1);
                        auto p0_world = posA + rotate(ornA, p0);
                        auto p0_B = rotate(conjugate(ornB), p0_world - posB);

                        auto idx = result.num_points++;
                        result.point[idx].pivotA = {pivotA_x, p0.y, p0.z};
                        result.point[idx].pivotB = p0_B;
                        result.point[idx].normalB = axis.dir;
                        result.point[idx].distance = penetration;

                        if (num_points == 2) {
                            s1 = clamp_unit(s1);
                            auto p1 = lerp(v0_A, v1_A, s1);
                            auto p1_world = posA + rotate(ornA, p1);
                            auto p1_B = rotate(conjugate(ornB), p1_world - posB);

                            auto idx = result.num_points++;
                            result.point[idx].pivotA = {pivotA_x, p1.y, p1.z};
                            result.point[idx].pivotB = p1_B;
                            result.point[idx].normalB = axis.dir;
                            result.point[idx].distance = penetration;
                        }
                    }

                    if (num_edge_intersections == 0) {
                        // Either circle is contained in triangle or triangle is contained in circle.
                        // If the circle doesn't intersect the edges of the triangle and the distance
                        // between one vertex of the triangle and the center of the circle is smaller
                        // than the radius, it means the triangle is contained within the circle.
                        scalar t;
                        vector3 r;

                        if (closest_point_line(posA_in_B, axis.dir, vertices[0], t, r) < shA.radius * shA.radius) {
                            result.num_points = 3;
                            auto pivotA_x = shA.half_length * (axis.cyl_feature_index == 0 ? 1 : -1);

                            for(uint8_t i = 0; i < 3; ++i) {
                                auto vertex_B = vertices[i];
                                auto vertex_world = posB + rotate(ornB, vertex_B);
                                auto vertex_A = rotate(conjugate(ornA), vertex_world - posA);
                                vertex_A.x = pivotA_x;

                                result.point[i].pivotA = vertex_A;
                                result.point[i].pivotB = vertex_B;
                                result.point[i].normalB = axis.dir;
                                result.point[i].distance = penetration;
                            }
                        } else if (point_in_triangle(vertices, tri_normal, posA_in_B)) {
                            // If the circle doesn't intersect the edges of the triangle and its center
                            // is contained within the triangle, it means the entire circle is contained
                            // within the triangle.
                            result.num_points = 4;
                            auto multipliers = std::array<scalar, 4>{0, 1, 0, -1};
                            for(uint8_t i = 0; i < 4; ++i) {
                                auto pivotA_x = shA.half_length * (axis.cyl_feature_index == 0 ? 1 : -1);
                                auto pivotA = vector3{pivotA_x, 
                                                      shA.radius * multipliers[i], 
                                                      shA.radius * multipliers[(i + 1) % 4]};
                                auto pivotA_world = posA + rotate(ornA, pivotA);
                                auto pivotB = rotate(conjugate(ornB), pivotA_world - posB);
                                result.point[i].pivotA = pivotA;
                                result.point[i].pivotB = pivotB;
                                result.point[i].normalB = axis.dir;
                                result.point[i].distance = penetration;
                            }
                        }
                    } else if (num_edge_intersections == 1) {
                        // Add extra points to create a stable base.
                        auto tangent = cross(tri_normal, edges[last_edge_index]);
                        auto other_vertex_idx = (last_edge_index + 2) % 3;
                        if (dot(tangent, vertices[other_vertex_idx] - vertices[last_edge_index]) < 0) {
                            tangent *= -1;
                        }

                        tangent = normalize(tangent);
                        auto cyl_axis = quaternion_x(ornA_in_B);
                        auto disc_center = posA_in_B + cyl_axis * shA.half_length * (axis.cyl_feature_index == 0 ? 1 : -1);
                        auto pivotA_in_B = disc_center + tangent * shA.radius;
                        auto pivotA_world = posB + rotate(ornB, pivotA_in_B);
                        auto pivotA = rotate(conjugate(ornA), pivotA_world - posA);
                        auto pivotB = project_plane(pivotA_in_B, vertices[0], tri_normal);
                        auto idx = result.num_points++;
                        result.point[idx].pivotA = pivotA;
                        result.point[idx].pivotB = pivotB;
                        result.point[idx].normalB = axis.dir;
                        result.point[idx].distance = penetration;
                    }
                    break;
                }
                break;
            }

            case CYLINDER_FEATURE_SIDE_EDGE: {
                // Segment-triangle intersection/containment test.
                auto cyl_axis = quaternion_x(ornA_in_B);
                auto c0 = posA_in_B + cyl_axis * shA.half_length;
                auto c1 = posA_in_B - cyl_axis * shA.half_length;
                auto c0_in_tri = point_in_triangle(vertices, tri_normal, c0);
                auto c1_in_tri = point_in_triangle(vertices, tri_normal, c1);

                if (c0_in_tri && c1_in_tri) {
                    result.num_points = 2;

                    auto p0 = c0 - tri_normal * shA.radius;
                    auto p0_world = posB + rotate(ornB, p0);
                    auto p0_A = rotate(conjugate(ornA), p0_world - posA);
                    result.point[0].pivotA = p0_A;
                    result.point[0].pivotB = project_plane(c0, vertices[0], tri_normal);
                    result.point[0].normalB = axis.dir;
                    result.point[0].distance = penetration;

                    auto p1 = c1 - tri_normal * shA.radius;
                    auto p1_world = posB + rotate(ornB, p1);
                    auto p1_A = rotate(conjugate(ornA), p1_world - posA);
                    result.point[1].pivotA = p1_A;
                    result.point[1].pivotB = project_plane(c1, vertices[0], tri_normal);
                    result.point[1].normalB = axis.dir;
                    result.point[1].distance = penetration;
                }
                
                break;
            }
            case CYLINDER_FEATURE_CIRCLE_EDGE: {
                auto pivotA_world = posB + rotate(ornB, axis.pivotA);
                auto pivotA = rotate(conjugate(ornA), pivotA_world - posA);
                auto pivotB = project_plane(axis.pivotA, vertices[0], tri_normal);
                auto idx = result.num_points++;
                result.point[idx].pivotA = pivotA;
                result.point[idx].pivotB = pivotB;
                result.point[idx].normalB = axis.dir;
                result.point[idx].distance = axis.distance;
                break;
            }
            }

            
        }
    });

    return result;
}

}