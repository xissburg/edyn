#include "edyn/collision/collide.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/math/math.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/math/vector2.hpp"
#include "edyn/math/vector2_3_util.hpp"
#include "edyn/shapes/box_shape.hpp"
#include "edyn/shapes/cylinder_shape.hpp"

namespace edyn {

struct cyl_box_separating_axis {
    vector3 dir;
    cylinder_feature featureA;
    box_feature featureB;
    size_t feature_indexA;
    size_t feature_indexB;
    vector3 pivotA;
    vector3 pivotB;
    scalar distance;
};

collision_result collide(const cylinder_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const box_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold) {
    // Cylinder-Box SAT. Normal of 3 faces of B, normal of cylinder caps of A
    // which is the cylinder main axis, 3 cross products between the axes of the
    // basis of the box and the cylinder axis mainly for the cylinder side edge
    // vs box edge collisions, 8 point-line closest point calculations between
    // the cylinder axis and the box vertices mainly for cylinder side edge vs
    // box vertex collisions, and 24 circle-segment closest point
    // normal between edges of B and cap edges of A.
    std::array<cyl_box_separating_axis, 3 + 1 + 3 + 8 + 24> sep_axes;
    size_t axis_idx = 0;

    auto box_axes = std::array<vector3, 3>{
        quaternion_x(ornB),
        quaternion_y(ornB),
        quaternion_z(ornB)
    };

    const auto cyl_axis = quaternion_x(ornA);
    const auto face_center_pos = posA + cyl_axis * shA.half_length;
    const auto face_center_neg = posA - cyl_axis * shA.half_length;

    auto posA_in_B = rotate(conjugate(ornB), posA - posB);
    auto ornA_in_B = conjugate(ornB) * ornA;

    // Box faces.
    for (size_t i = 0; i < 3; ++i) {
        auto &axisB = box_axes[i];
        auto &axis = sep_axes[axis_idx++];
        axis.featureB = box_feature::face;

        // Make dir point towards A.
        if (dot(posA - posB, axisB) > 0) {
            axis.feature_indexB = i * 2; // Positive face along axis.
            axis.dir = axisB; // Point towards A.
        } else {
            axis.feature_indexB = i * 2 + 1; // Negative face along axis.
            axis.dir = -axisB; // Point towards A.
        }

        shA.support_feature(posA, ornA, posB, -axis.dir, 
                            axis.featureA, axis.feature_indexA, 
                            axis.pivotA, axis.distance, threshold);
        axis.distance = -(shB.half_extents[i] + axis.distance);
        axis.pivotB = axis.pivotA - axis.dir * axis.distance;
    }

    // Cylinder cap faces.
    {
        auto &axis = sep_axes[axis_idx++];
        axis.featureA = cylinder_feature::face;
        vector3 face_center;

        // Make dir point towards A.
        if (dot(posA - posB, cyl_axis) < 0) {
            axis.dir = -cyl_axis;
            axis.feature_indexA = 0;
            face_center = face_center_pos;
        } else {
            axis.dir = cyl_axis;
            axis.feature_indexA = 1;
            face_center = face_center_neg;
        }

        shB.support_feature(posB, ornB, face_center, axis.dir, 
                            axis.featureB, axis.feature_indexB, 
                            axis.distance, threshold);
        axis.distance = -axis.distance;
    }

    // Box edges vs cylinder side edges.
    for (size_t i = 0; i < 3; ++i) {
        auto dir = cross(box_axes[i], cyl_axis);
        auto dir_len_sqr = length_sqr(dir);

        if (!(dir_len_sqr > EDYN_EPSILON)) {
            continue;
        }

        dir /= std::sqrt(dir_len_sqr);

        // Make it point towards A.
        if (dot(posA - posB, dir) < 0) {
            dir *= -1;
        }

        auto &axis = sep_axes[axis_idx++];
        axis.dir = dir;
        axis.featureA = cylinder_feature::side_edge;

        shB.support_feature(posB, ornB, posA, axis.dir, 
                            axis.featureB, axis.feature_indexB, 
                            axis.distance, threshold);
        axis.distance = -(shA.radius + axis.distance);
    }

    // Box vertices vs cylinder side edges.
    for (size_t i = 0; i < get_box_num_features(box_feature::vertex); ++i) {
        auto vertex = shB.get_vertex(i, posB, ornB);
        vector3 closest; scalar t;
        closest_point_line(face_center_neg, cyl_axis, vertex, t, closest);

        auto dir = closest - vertex;
        auto dir_len_sqr = length_sqr(dir);

        if (!(dir_len_sqr > EDYN_EPSILON)) {
            continue;
        }

        dir /= std::sqrt(dir_len_sqr);

        // Make it point towards A.
        if (dot(posA - posB, dir) < 0) {
            dir *= -1;
        }

        auto &axis = sep_axes[axis_idx++];
        axis.dir = dir;
        axis.featureA = cylinder_feature::side_edge;

        shB.support_feature(posB, ornB, posA, axis.dir, 
                            axis.featureB, axis.feature_indexB, 
                            axis.distance, threshold);
        axis.distance = -(shA.radius + axis.distance);
    }

    // Cylinder cap edges.
    for (size_t i = 0; i < 2; ++i) {
        auto face_center = i == 0 ? face_center_neg : face_center_pos;

        for (size_t j = 0; j < get_box_num_features(box_feature::edge); ++j) {
            auto edge_vertices = shB.get_edge(j, posB, ornB);

            // Find closest point between circle and triangle edge segment. 
            size_t num_points;
            scalar s0, s1;
            vector3 cc0, cl0, cc1, cl1;
            vector3 normal;
            closest_point_circle_line(face_center, ornA, shA.radius, 
                                      edge_vertices[0], edge_vertices[1], 
                                      num_points, s0, cc0, cl0, s1, cc1, cl1, 
                                      normal, threshold);
            
            if (s0 > 0 && s0 < 1) {
                // Make it point towards A.
                if (dot(posA - posB, normal) < 0) {
                    normal *= -1;
                }

                auto &axis = sep_axes[axis_idx++];
                axis.dir = normal;

                scalar projA, projB;
                shA.support_feature(posA, ornA, posA, -normal, 
                                    axis.featureA, axis.feature_indexA, 
                                    axis.pivotA, projA, threshold);
                shB.support_feature(posB, ornB, posA, normal, 
                                    axis.featureB, axis.feature_indexB, 
                                    projB, threshold);
                axis.distance = -(projA + projB);
                axis.pivotB = lerp(edge_vertices[0], edge_vertices[1], s0);
            }
        }
    }

    auto greatest_distance = -EDYN_SCALAR_MAX;
    size_t sep_axis_idx;

    for (size_t i = 0; i < axis_idx; ++i) {
        auto &sep_axis = sep_axes[i];
        
        if (sep_axis.distance > greatest_distance) {
            greatest_distance = sep_axis.distance;
            sep_axis_idx = i;
        }
    }

    auto &sep_axis = sep_axes[sep_axis_idx];

    if (sep_axis.distance > threshold) {
        return {};
    }

    auto result = collision_result{};
    auto normalB = rotate(conjugate(ornB), sep_axis.dir);

    switch (sep_axis.featureA) {
    case cylinder_feature::face: {
        size_t num_vertices_in_face = 0;
        auto num_feature_vertices = get_box_feature_num_vertices(sep_axis.featureB);
        std::array<vector3, 4> vertices_in_B;

        switch (sep_axis.featureB) {
        case box_feature::face: {
            auto vs = shB.get_face(sep_axis.feature_indexB);
            std::copy(vs.begin(), vs.end(), vertices_in_B.begin());
        }
        break;
        case box_feature::edge: {
            auto vs = shB.get_edge(sep_axis.feature_indexB);
            std::copy(vs.begin(), vs.end(), vertices_in_B.begin());
        }
        break;
        case box_feature::vertex:
            vertices_in_B[0] = shB.get_vertex(sep_axis.feature_indexB);
        }            

        // Check if box feature vertices are inside a cylinder cap face (by checking
        // if its distance from the cylinder axis is smaller than the cylinder radius).
        for (size_t i = 0; i < num_feature_vertices; ++i) {
            auto &vertex_in_B = vertices_in_B[i];
            auto vertex_world = posB + rotate(ornB, vertex_in_B);
            vector3 closest; scalar t;
            auto dist_sqr = closest_point_line(posA, cyl_axis, vertex_world, t, closest);

            if (dist_sqr <= shA.radius * shA.radius) {
                auto vertex_in_A = to_object_space(vertex_world, posA, ornA);
                auto vertex_proj_in_A = vertex_in_A;
                vertex_proj_in_A.x = shA.half_length * (sep_axis.feature_indexA == 0 ? 1 : -1);
                result.maybe_add_point({vertex_proj_in_A, vertex_in_B, normalB, sep_axis.distance});

                ++num_vertices_in_face;
            }
        }
    
        // If not all vertices of the box feature are contained in the cylinder
        // cap face, there could be edge intersections or if it is a `box_feature::face`
        // then the cylinder cap face could be contained within the box face.
        size_t num_edge_intersections = 0;
        size_t num_edges_to_check = 0;
        std::pair<vector3, vector3> last_edge;

        if (sep_axis.featureB == box_feature::edge && 
            num_vertices_in_face < 2) {
            num_edges_to_check = 1;
        } else if (sep_axis.featureB == box_feature::face && 
                   num_vertices_in_face < 4) {
            num_edges_to_check = 4;
        }

        // Check if circle and box edges intersect.
        for (size_t i = 0; i < num_edges_to_check; ++i) {
            // Transform vertices to `shA` (cylinder) space. The cylinder axis
            // is the x-axis.
            auto v0_B = vertices_in_B[i];
            auto v1_B = vertices_in_B[(i + 1) % 4];

            auto v0_w = to_world_space(v0_B, posB, ornB);
            auto v1_w = to_world_space(v1_B, posB, ornB);

            auto v0_A = to_object_space(v0_w, posA, ornA);
            auto v1_A = to_object_space(v1_w, posA, ornA);

            scalar s0, s1;
            auto num_points = intersect_line_circle(to_vector2_zy(v0_A), 
                                                    to_vector2_zy(v1_A), 
                                                    shA.radius, s0, s1);

            if (num_points > 0) {
                ++num_edge_intersections;
                last_edge = std::make_pair(v0_w, v1_w);

                s0 = clamp_unit(s0);
                auto pivotA_x = shA.half_length * (sep_axis.feature_indexA == 0 ? 1 : -1);
                auto pivotA = lerp(v0_A, v1_A, s0);
                pivotA.x = pivotA_x;
                auto pivotB = lerp(v0_B, v1_B, s0);
                result.maybe_add_point({pivotA, pivotB, normalB, sep_axis.distance});

                if (num_points == 2) {
                    s1 = clamp_unit(s1);
                    auto pivotA = lerp(v0_A, v1_A, s1);
                    pivotA.x = pivotA_x;
                    auto pivotB = lerp(v0_B, v1_B, s1);
                    result.maybe_add_point({pivotA, pivotB, normalB, sep_axis.distance});
                }
            }
        }

        if (sep_axis.featureB == box_feature::face) {
            // If no vertex is contained in the circular face nor there is
            // any edge intersection, it means the circle could be contained
            // in the box face.
            if (num_vertices_in_face == 0 && num_edge_intersections == 0) {
                // Check if cylinder face center is in quad.
                if (point_in_quad(posA_in_B, vertices_in_B, sep_axis.dir)) {
                    auto multipliers = std::array<scalar, 4>{0, 1, 0, -1};
                    for(int i = 0; i < 4; ++i) {
                        auto pivotA_x = shA.half_length * (sep_axis.feature_indexA == 0 ? 1 : -1);
                        auto pivotA = vector3{pivotA_x, 
                                              shA.radius * multipliers[i], 
                                              shA.radius * multipliers[(i + 1) % 4]};
                        auto pivotB = posA_in_B + rotate(ornA_in_B, pivotA);
                        pivotB = project_plane(pivotB, vertices_in_B[0], normalB);
                        result.maybe_add_point({pivotA, pivotB, normalB, sep_axis.distance});
                    }
                }
            } else if (num_edge_intersections == 1) {
                // If it intersects a single edge, only two contact points have been added,
                // thus add extra points to create a stable base.
                auto edge_in_A = std::make_pair(to_vector2_zy(to_object_space(last_edge.first, posA, ornA)),
                                                to_vector2_zy(to_object_space(last_edge.second, posA, ornA)));
                auto edge_dir = edge_in_A.second - edge_in_A.first;
                auto tangent = orthogonal(edge_dir);

                // Make tangent point towards box face.
                auto box_face_center = to_vector2_zy(to_object_space(posB, posA, ornA));
                if (dot(tangent, box_face_center) < 0) {
                    tangent *= -1;
                }

                tangent = normalize(tangent);

                auto pivotA_x = shA.half_length * (sep_axis.feature_indexA == 0 ? 1 : -1);
                auto pivotA = vector3{pivotA_x, tangent.y * shA.radius, tangent.x * shA.radius};
                // Transform pivotA into box space and project onto box face.
                auto pivotB = posA_in_B + rotate(ornA_in_B, pivotA);
                pivotB = project_plane(pivotB, vertices_in_B[0], normalB);
                result.maybe_add_point({pivotA, pivotB, normalB, sep_axis.distance});
            }
        }
    }
    break;

    case cylinder_feature::side_edge: {
        switch (sep_axis.featureB) {
        case box_feature::face: {
            auto face_normal = shB.get_face_normal(sep_axis.feature_indexB, ornB);
            auto face_vertices = shB.get_face(sep_axis.feature_indexB, posB, ornB);
            std::array<vector3, 2> edge_vertices;
            edge_vertices[0] = face_center_neg - sep_axis.dir * shA.radius;
            edge_vertices[1] = face_center_pos - sep_axis.dir * shA.radius;

            std::array<vector3, 4> face_tangents;
            for (int i = 0; i < 4; ++i) {
                auto &v0 = face_vertices[i];
                auto &v1 = face_vertices[(i + 1) % 4];
                face_tangents[i] = cross(face_normal, v1 - v0);
            }

            // Check if edge vertices are inside face.
            for (int i = 0; i < 2; ++i) {
                if (dot(edge_vertices[i] - face_vertices[0], face_tangents[0]) > 0 &&
                    dot(edge_vertices[i] - face_vertices[1], face_tangents[1]) > 0 &&
                    dot(edge_vertices[i] - face_vertices[2], face_tangents[2]) > 0 &&
                    dot(edge_vertices[i] - face_vertices[3], face_tangents[3]) > 0) {
                    // Edge's vertex is inside face.
                    auto pivot_face = project_plane(edge_vertices[i], face_vertices[0], face_normal);
                    auto pivotA = to_object_space(edge_vertices[i], posA, ornA);
                    auto pivotB = to_object_space(pivot_face, posB, ornB);
                    result.add_point({pivotA, pivotB, normalB, sep_axis.distance});
                }
            }

            // If both vertices are not inside the face then perform edge intersection tests.
            if (result.num_points < 2) {
                auto face_center = shB.get_face_center(sep_axis.feature_indexB, posB, ornB);
                auto face_basis = shB.get_face_basis(sep_axis.feature_indexB, ornB);
                auto half_extents = shB.get_face_half_extents(sep_axis.feature_indexB);

                auto e0 = to_object_space(edge_vertices[0], face_center, face_basis);
                auto e1 = to_object_space(edge_vertices[1], face_center, face_basis);
                auto p0 = vector2{e0.x, e0.z};
                auto p1 = vector2{e1.x, e1.z};

                scalar s[2];
                auto num_points = intersect_line_aabb(p0, p1, -half_extents, half_extents, s[0], s[1]);

                for (size_t i = 0; i < num_points; ++i) {
                    if (s[i] >= 0 && s[i] <= 1) {
                        auto edge_pivot = lerp(edge_vertices[0], edge_vertices[1], s[i]);
                        auto face_pivot = project_plane(edge_pivot, face_center, sep_axis.dir);
                        auto pivotA = to_object_space(edge_pivot, posA, ornA);
                        auto pivotB = to_object_space(face_pivot, posB, ornB);
                        result.add_point({pivotA, pivotB, normalB, sep_axis.distance});
                    }
                }
            }
        }
        break;
        case box_feature::edge: {
            auto box_edge = shB.get_edge(sep_axis.feature_indexB, posB, ornB);
            scalar s[2], t[2];
            vector3 pA[2], pB[2];
            size_t num_points = 0;
            closest_point_segment_segment(face_center_neg, face_center_pos,
                                          box_edge[0], box_edge[1],
                                          s[0], t[0], pA[0], pB[0], &num_points,
                                          &s[1], &t[1], &pA[1], &pB[1]);

            for (size_t i = 0; i < num_points; ++i) {
                auto pivotA_world = pA[i] - sep_axis.dir * shA.radius;
                auto pivotB_world = pB[i];
                auto pivotA = to_object_space(pivotA_world, posA, ornA);
                auto pivotB = to_object_space(pivotB_world, posB, ornB);
                result.add_point({pivotA, pivotB, normalB, sep_axis.distance});
            }
        }
        break;
        case box_feature::vertex: {
            auto pivotB = shB.get_vertex(sep_axis.feature_indexB);
            auto pivotB_world = to_world_space(pivotB, posB, ornB);
            vector3 closest; scalar t;
            closest_point_segment(face_center_pos, face_center_neg, pivotB_world, t, closest);

            if (!(t < 0) && !(t > 1)) {
                auto pivotA_world = closest - sep_axis.dir * shA.radius;
                auto pivotA = to_object_space(pivotA_world, posA, ornA);
                result.add_point({pivotA, pivotB, normalB, sep_axis.distance});
            }
        }
        }  
    }
    break;

    case cylinder_feature::cap_edge: {
        auto pivotA = to_object_space(sep_axis.pivotA, posA, ornA);
        auto pivotB = to_object_space(sep_axis.pivotB, posB, ornB);
        result.maybe_add_point({pivotA, pivotB, normalB, sep_axis.distance});
    }
    }

    return result;
}

}