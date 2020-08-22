#include "edyn/collision/collide.hpp"
#include "edyn/math/math.hpp"

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
    // Cylinder-Box SAT. Normal of 3 faces of B, normal of shA cap of A, 
    // 3 cross-products between edges of B and axis of A, 24 circle-segment 
    // closest point normal between edges of B and cap edges of A.
    std::array<cyl_box_separating_axis, 3 + 1 + 3 + 24> sep_axes;
    size_t axis_idx = 0;

    auto box_axes = std::array<vector3, 3>{
        quaternion_x(ornB),
        quaternion_y(ornB),
        quaternion_z(ornB)
    };

    auto cyl_axis = quaternion_x(ornA);

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
        std::array<vector3, 4> vertices;

        switch (sep_axis.featureB) {
        case box_feature::face: {
            auto vs = shB.get_face(sep_axis.feature_indexB);
            std::copy(vs.begin(), vs.end(), vertices.begin());
        }
        break;
        case box_feature::edge: {
            auto vs = shB.get_edge(sep_axis.feature_indexB);
            std::copy(vs.begin(), vs.end(), vertices.begin());
        }
        break;
        case box_feature::vertex:
            vertices[0] = shB.get_vertex(sep_axis.feature_indexB);
        }            

        // Check if box feature vertices are inside a cylinder cap face (by checking
        // if its distance from the cylinder axis is smaller than the cylinder radius).
        for (size_t i = 0; i < num_feature_vertices; ++i) {
            auto &vertex = vertices[i];
            vector3 closest; scalar t;
            auto dist_sqr = closest_point_line(posA, cyl_axis, vertex, t, closest);

            if (dist_sqr <= shA.radius * shA.radius) {
                auto vertex_in_A = to_object_space(vertex, posA, ornA);
                auto vertex_proj_in_A = vertex_in_A;
                vertex_proj_in_A.x = shA.half_length * (sep_axis.feature_indexA == 0 ? 1 : -1);
                result.maybe_add_point({vertex_proj_in_A, vertex, sep_axis.dir, sep_axis.distance});

                ++num_vertices_in_face;
            }
        }
    
        // If not all vertices of the box feature are contained in the cylinder
        // cap face, there could be edge intersections or if it is a `box_feature::face`
        // then the cap face could be contained within the box face.
        size_t num_edge_intersections = 0;
        size_t num_edges_to_check = 0;

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
            auto v0 = vertices[i];
            auto v0_A = to_object_space(v0, posA, ornA);

            auto v1 = vertices[(i + 1) % 4];
            auto v1_A = to_object_space(v1, posA, ornA);

            scalar s0, s1;
            auto num_points = intersect_line_circle(v0_A.z, v0_A.y, 
                                                    v1_A.z, v1_A.y, 
                                                    shA.radius, s0, s1);

            if (num_points > 0) {
                ++num_edge_intersections;

                s0 = clamp_unit(s0);
                auto pivotA_x = shA.half_length * (sep_axis.feature_indexA == 0 ? 1 : -1);
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

        if (sep_axis.featureB == box_feature::face) {
            // If no vertex is contained in the circular face nor there is
            // any edge intersection, it means the circle could be contained
            // in the box face.
            if (num_vertices_in_face == 0 && num_edge_intersections == 0) {
                // Check if face center is in quad.
                if (point_in_quad(posA, vertices, sep_axis.dir)) {
                    auto multipliers = std::array<scalar, 4>{0, 1, 0, -1};
                    for(int i = 0; i < 4; ++i) {
                        auto pivotA_x = shA.half_length * (sep_axis.feature_indexA == 0 ? 1 : -1);
                        auto pivotA = vector3{pivotA_x, 
                                              shA.radius * multipliers[i], 
                                              shA.radius * multipliers[(i + 1) % 4]};
                        auto pivotB = posA + rotate(ornA, pivotA);
                        pivotB = project_plane(pivotB, vertices[0], sep_axis.dir);
                        result.maybe_add_point({pivotA, pivotB, sep_axis.dir, sep_axis.distance});
                    }
                }
            } else if (num_edge_intersections == 1) {
                // If it intersects a single edge, only two contact points have been added,
                // thus add extra points to create a stable base.
                /* auto tangent = cross(sep_axis.dir, edges[last_edge_index]);
                auto other_vertex_idx = (last_edge_index + 2) % 3;
                if (dot(tangent, vertices[other_vertex_idx] - vertices[last_edge_index]) < 0) {
                    tangent *= -1;
                }

                tangent = normalize(tangent);
                auto disc_center = sep_axis.cyl_feature_index == 0 ? disc_center_pos : disc_center_neg;
                auto pivotA_in_B = disc_center + tangent * cylinder.radius;
                auto pivotA = to_object_space(pivotA_in_B, posA, ornA);
                auto pivotB = project_plane(pivotA_in_B, vertices[0], tri_normal);
                result.maybe_add_point({pivotA, pivotB, sep_axis.dir, sep_axis.distance}); */
            }
        }
    }
    break;

    case cylinder_feature::edge: {

    }
    break;

    case cylinder_feature::cap_edge: {
        switch (sep_axis.featureB) {
        case box_feature::face: {
            auto pivotA = to_object_space(sep_axis.pivotA, posA, ornA);
            auto pivotB = to_object_space(sep_axis.pivotB, posB, ornB);
            result.maybe_add_point({pivotA, pivotB, sep_axis.dir, sep_axis.distance});
        }
        break;

        }
    }
    }

    return result;
}

}