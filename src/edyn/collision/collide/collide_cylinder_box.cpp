#include "edyn/collision/collide.hpp"
#include "edyn/config/constants.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/math/math.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/math/transform.hpp"
#include "edyn/math/vector2.hpp"
#include "edyn/math/vector2_3_util.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/shapes/box_shape.hpp"
#include "edyn/shapes/cylinder_shape.hpp"
#include "edyn/math/coordinate_axis.hpp"

namespace edyn {

void collide(const cylinder_shape &shA, const box_shape &shB,
             const collision_context &ctx, collision_result &result) {
    const auto &posA = ctx.posA;
    const auto &ornA = ctx.ornA;
    const auto &posB = ctx.posB;
    const auto &ornB = ctx.ornB;

    const auto box_axes = std::array<vector3, 3>{
        quaternion_x(ornB),
        quaternion_y(ornB),
        quaternion_z(ornB)
    };

    const auto cyl_axis = coordinate_axis_vector(shA.axis, ornA);
    const auto cyl_vertices = std::array<vector3, 2>{
        posA + cyl_axis * shA.half_length,
        posA - cyl_axis * shA.half_length
    };

    vector3 sep_axis;
    scalar distance = -EDYN_SCALAR_MAX;

    // Box faces.
    for (size_t i = 0; i < 3; ++i) {
        auto dir = box_axes[i];

        if (dot(posA - posB, dir) < 0) {
            dir *= -1; // Make dir point towards A.
        }

        auto projA = -shA.support_projection(posA, ornA, -dir);
        auto projB = dot(posB, dir) + shB.half_extents[i];
        auto dist = projA - projB;

        if (dist > distance) {
            distance = dist;
            sep_axis = dir;
        }
    }

    // Cylinder cap faces.
    {
        vector3 dir = cyl_axis;

        if (dot(posA - posB, dir) < 0) {
            dir *= -1; // Make dir point towards A.
        }

        auto projA = -(dot(posA, -dir) + shA.half_length);
        auto projB = shB.support_projection(posB, ornB, dir);
        auto dist = projA - projB;

        if (dist > distance) {
            distance = dist;
            sep_axis = dir;
        }
    }

    // Box edges vs cylinder side edges.
    for (size_t i = 0; i < 3; ++i) {
        auto dir = cross(box_axes[i], cyl_axis);

        if (!try_normalize(dir)) {
            continue;
        }

        if (dot(posA - posB, dir) < 0) {
            dir *= -1; // Make it point towards A.
        }

        auto projA = -shA.support_projection(posA, ornA, -dir);
        auto projB = shB.support_projection(posB, ornB, dir);
        auto dist = projA - projB;

        if (dist > distance) {
            distance = dist;
            sep_axis = dir;
        }
    }

    // Box vertices vs cylinder side edges.
    for (size_t i = 0; i < get_box_num_features(box_feature::vertex); ++i) {
        auto vertex = shB.get_vertex(i, posB, ornB);
        vector3 closest; scalar t;
        closest_point_line(posA, cyl_axis, vertex, t, closest);
        auto dir = closest - vertex;

        if (!try_normalize(dir)) {
            continue;
        }

        if (dot(posA - posB, dir) < 0) {
            dir *= -1; // Make it point towards A.
        }

        auto projA = -(dot(posA, -dir) + shA.radius);
        auto projB = shB.support_projection(posB, ornB, dir);
        auto dist = projA - projB;

        if (dist > distance) {
            distance = dist;
            sep_axis = dir;
        }
    }

    // Cylinder cap edges.
    for (size_t i = 0; i < 2; ++i) {
        auto circle_position = cyl_vertices[i];

        for (size_t j = 0; j < get_box_num_features(box_feature::edge); ++j) {
            auto edge_vertices = shB.get_edge(j, posB, ornB);

            // Find closest point between circle and triangle edge segment.
            size_t num_points;
            scalar s[2];
            vector3 closest_circle[2];
            vector3 closest_line[2];
            vector3 dir;
            closest_point_circle_line(circle_position, ornA, shA.radius, shA.axis,
                                      edge_vertices[0], edge_vertices[1], num_points,
                                      s[0], closest_circle[0], closest_line[0],
                                      s[1], closest_circle[1], closest_line[1],
                                      dir, support_feature_tolerance);

            // If there are two closest points, it means the segment is parallel
            // to the plane of the circle, which means the separating axis would
            // be a cylinder cap face normal which was already handled.
            if (num_points == 2) {
                continue;
            }

            if (dot(posA - posB, dir) < 0) {
                dir *= -1; // Make it point towards A.
            }

            auto projA = -shA.support_projection(posA, ornA, -dir);
            auto projB = shB.support_projection(posB, ornB, dir);
            auto dist = projA - projB;

            if (dist > distance) {
                distance = dist;
                sep_axis = dir;
            }
        }
    }

    if (distance > ctx.threshold) {
        return;
    }

    cylinder_feature featureA;
    size_t feature_indexA;
    shA.support_feature(posA, ornA, -sep_axis, featureA, feature_indexA,
                        support_feature_tolerance);

    box_feature featureB;
    size_t feature_indexB;
    shB.support_feature(posB, ornB, sep_axis, featureB, feature_indexB,
                        support_feature_tolerance);

    collision_result::collision_point point;
    point.normal = sep_axis;
    point.distance = distance;
    point.featureA = {featureA, feature_indexA};
    point.featureB = {featureB, feature_indexB};

    // Index of vector element in cylinder object space that represents the
    // cylinder axis followed by the indices of the elements of the axes
    // orthogonal to the cylinder axis.
    auto cyl_ax_idx = static_cast<std::underlying_type_t<coordinate_axis>>(shA.axis);
    auto cyl_ax_idx_ortho0 = (cyl_ax_idx + 1) % 3;
    auto cyl_ax_idx_ortho1 = (cyl_ax_idx + 2) % 3;

    if (featureA == cylinder_feature::face && featureB == box_feature::face) {
        auto sign_faceA = to_sign(feature_indexA == 0);
        auto verticesB_local = shB.get_face(feature_indexB);
        std::array<vector3, 4> verticesB_world;

        for (size_t i = 0; i < 4; ++i) {
            verticesB_world[i] = to_world_space(verticesB_local[i], posB, ornB);
        }

        point.normal_attachment = contact_normal_attachment::normal_on_B;

        size_t num_edge_intersections = 0;
        std::array<vector3, 2> last_edge;

        // Check if circle and face edges intersect.
        for (size_t vertex_idx = 0; vertex_idx < 4; ++vertex_idx) {
            auto next_vertex_idx = (vertex_idx + 1) % 4;
            // Transform vertices to `shA` (cylinder) space. The cylinder axis
            // is the x-axis.
            auto v0w = verticesB_world[vertex_idx];
            auto v1w = verticesB_world[next_vertex_idx];

            auto v0A = to_object_space(v0w, posA, ornA);
            auto v1A = to_object_space(v1w, posA, ornA);
            // Project points onto plane orthogonal to cylinder axis.
            auto v0A_proj = vector2{v0A[cyl_ax_idx_ortho0], v0A[cyl_ax_idx_ortho1]};
            auto v1A_proj = vector2{v1A[cyl_ax_idx_ortho0], v1A[cyl_ax_idx_ortho1]};

            scalar s[2];
            auto num_points = intersect_line_circle(v0A_proj, v1A_proj,
                                                    shA.radius, s[0], s[1]);

            if (num_points == 0) {
                continue;
            }

            // Single point outside the segment range.
            if (num_points == 1 && (s[0] < 0 || s[0] > 1)) {
                continue;
            }

            // If both s'es are either below zero or above one, it means the
            // line intersects the circle, but the segment doesn't.
            if (num_points == 2 && ((s[0] < 0 && s[1] < 0) || (s[0] > 1 && s[1] > 1))) {
                continue;
            }

            ++num_edge_intersections;
            last_edge[0] = v0w;
            last_edge[1] = v1w;

            auto v0B = verticesB_local[vertex_idx];
            auto v1B = verticesB_local[next_vertex_idx];
            auto pivotA_axis = shA.half_length * sign_faceA;

            for (size_t pt_idx = 0; pt_idx < num_points; ++pt_idx) {
                auto t = s[pt_idx];
                // Avoid adding the same point twice.
                if (!(t < 1)) continue;

                auto u = clamp_unit(t);
                point.pivotA = lerp(v0A, v1A, u);
                point.pivotB = lerp(v0B, v1B, u);
                point.distance = (point.pivotA[cyl_ax_idx] - pivotA_axis) * sign_faceA;
                point.pivotA[cyl_ax_idx] = pivotA_axis; // Project onto cyl face.
                result.maybe_add_point(point);
            }
        }

        // If there are no edge intersections, it means the circle could be
        // contained in the box face.
        const auto posA_in_B = to_object_space(posA, posB, ornB);
        const auto ornA_in_B = conjugate(ornB) * ornA;
        auto face_normal_local = shB.get_face_normal(feature_indexB);

        if (num_edge_intersections == 0) {
            // Check if cylinder face center is in quad.
            if (point_in_polygonal_prism(verticesB_local, face_normal_local, posA_in_B)) {
                auto multipliers = std::array<scalar, 4>{0, 1, 0, -1};
                for(int i = 0; i < 4; ++i) {
                    auto j = (i + 1) % 4;
                    point.pivotA[cyl_ax_idx] = shA.half_length * sign_faceA;
                    point.pivotA[cyl_ax_idx_ortho0] = shA.radius * multipliers[i];
                    point.pivotA[cyl_ax_idx_ortho1] = shA.radius * multipliers[j];
                    auto pivotA_in_B = to_world_space(point.pivotA, posA_in_B, ornA_in_B);
                    point.distance = dot(pivotA_in_B - verticesB_local[0], face_normal_local);
                    point.pivotB = project_plane(pivotA_in_B, verticesB_local[0], face_normal_local);
                    result.maybe_add_point(point);
                }
            }
        } else if (num_edge_intersections == 1) {
            // If it intersects a single edge, only two contact points have
            // been added, thus add extra points to create a stable base.
            std::array<vector2, 2> edge_in_A;

            for (size_t i = 0; i < last_edge.size(); ++i) {
                auto last_edge_local = to_object_space(last_edge[i], posA, ornA);
                edge_in_A[i] = vector2{last_edge_local[cyl_ax_idx_ortho0], last_edge_local[cyl_ax_idx_ortho1]};
            }

            auto edge_dir = edge_in_A[1] - edge_in_A[0];
            auto tangent = normalize(orthogonal(edge_dir));

            auto posB_in_A = to_object_space(posB, posA, ornA);
            auto box_face_center = vector2{posB_in_A[cyl_ax_idx_ortho0], posB_in_A[cyl_ax_idx_ortho1]};

            // Make tangent point towards box face.
            if (dot(tangent, box_face_center) < 0) {
                tangent *= -1;
            }

            point.pivotA[cyl_ax_idx] = shA.half_length * to_sign(feature_indexA == 0);
            point.pivotA[cyl_ax_idx_ortho0] = tangent.x * shA.radius;
            point.pivotA[cyl_ax_idx_ortho1] = tangent.y * shA.radius;
            // Transform pivotA into box space and project onto box face.
            auto pivotA_in_B = to_world_space(point.pivotA, posA_in_B, ornA_in_B);
            point.pivotB = project_plane(pivotA_in_B, verticesB_local[0], face_normal_local);
            point.distance = dot(pivotA_in_B - verticesB_local[0], face_normal_local);
            result.maybe_add_point(point);
        }
    } else if (featureA == cylinder_feature::face && featureB == box_feature::edge) {
        auto verticesB_local = shB.get_edge(feature_indexB);
        std::array<vector3, 2> verticesB_world;

        for (size_t i = 0; i < 2; ++i) {
            verticesB_world[i] = to_world_space(verticesB_local[i], posB, ornB);
        }

        point.normal_attachment = contact_normal_attachment::normal_on_A;

        // Check if circle and edge intersect.
        // Transform vertices to `shA` (cylinder) space. The cylinder axis
        // is the x-axis.
        auto v0A = to_object_space(verticesB_world[0], posA, ornA);
        auto v1A = to_object_space(verticesB_world[1], posA, ornA);
        // Project points onto plane orthogonal to cylinder axis.
        auto v0A_proj = vector2{v0A[cyl_ax_idx_ortho0], v0A[cyl_ax_idx_ortho1]};
        auto v1A_proj = vector2{v1A[cyl_ax_idx_ortho0], v1A[cyl_ax_idx_ortho1]};

        scalar s[2];
        auto num_points = intersect_line_circle(v0A_proj, v1A_proj,
                                                shA.radius, s[0], s[1]);

        auto sign_faceA = to_sign(feature_indexA == 0);
        auto pivotA_axis = shA.half_length * sign_faceA;

        for (size_t pt_idx = 0; pt_idx < num_points; ++pt_idx) {
            auto t = clamp_unit(s[pt_idx]);
            point.pivotA = lerp(v0A, v1A, t);
            point.distance = (point.pivotA[cyl_ax_idx] - pivotA_axis) * sign_faceA;
            point.pivotA[cyl_ax_idx] = pivotA_axis;
            point.pivotB = lerp(verticesB_local[0], verticesB_local[1], t);
            result.maybe_add_point(point);
        }
    } else if (featureA == cylinder_feature::face && featureB == box_feature::vertex) {
        auto sign_faceA = to_sign(feature_indexA == 0);
        point.pivotB = shB.get_vertex(feature_indexB);
        auto pivotB_world = to_world_space(point.pivotB, posB, ornB);

        // Only insert point if it is inside face.
        if (!(distance_sqr_line(posA, cyl_axis, pivotB_world) > square(shA.radius))) {
            auto pivotA_axis = shA.half_length * sign_faceA;
            point.pivotA = to_object_space(pivotB_world, posA, ornA);
            point.distance = (point.pivotA[cyl_ax_idx] - pivotA_axis) * sign_faceA;
            point.pivotA[cyl_ax_idx] = pivotA_axis; // Project onto face.
            point.normal_attachment = contact_normal_attachment::normal_on_A;
            result.maybe_add_point(point);
        }
    } else if (featureA == cylinder_feature::side_edge && featureB == box_feature::face) {
        auto face_normal = shB.get_face_normal(feature_indexB, ornB);
        auto face_vertices = shB.get_face(feature_indexB, posB, ornB);

        point.normal_attachment = contact_normal_attachment::normal_on_B;

        std::array<vector3, 2> edge_vertices;
        edge_vertices[0] = cyl_vertices[0] - sep_axis * shA.radius;
        edge_vertices[1] = cyl_vertices[1] - sep_axis * shA.radius;

        // Perform edge intersection tests.
        auto face_center = shB.get_face_center(feature_indexB, posB, ornB);
        auto face_basis = shB.get_face_basis(feature_indexB, ornB);
        auto half_extents = shB.get_face_half_extents(feature_indexB);

        auto e0 = to_object_space(edge_vertices[0], face_center, face_basis);
        auto e1 = to_object_space(edge_vertices[1], face_center, face_basis);
        auto p0 = to_vector2_xz(e0);
        auto p1 = to_vector2_xz(e1);

        scalar s[2];
        auto num_points = intersect_line_aabb(p0, p1, -half_extents, half_extents, s[0], s[1]);

        for (size_t i = 0; i < num_points; ++i) {
            auto t = clamp_unit(s[i]); // Keep points within segment.
            auto edge_pivot = lerp(edge_vertices[0], edge_vertices[1], t);
            point.distance = dot(edge_pivot - face_vertices[0], face_normal);
            auto pivot_on_face = edge_pivot - face_normal * point.distance;
            point.pivotA = to_object_space(edge_pivot, posA, ornA);
            point.pivotB = to_object_space(pivot_on_face, posB, ornB);
            result.add_point(point);
        }
    } else if (featureA == cylinder_feature::side_edge && featureB == box_feature::edge) {
        point.normal_attachment = contact_normal_attachment::none;
        auto box_edge = shB.get_edge(feature_indexB, posB, ornB);
        scalar s[2], t[2];
        vector3 closestA[2], closestB[2];
        size_t num_points = 0;
        closest_point_segment_segment(cyl_vertices[0], cyl_vertices[1],
                                      box_edge[0], box_edge[1],
                                      s[0], t[0], closestA[0], closestB[0], &num_points,
                                      &s[1], &t[1], &closestA[1], &closestB[1]);

        for (size_t i = 0; i < num_points; ++i) {
            auto pivotA_world = closestA[i] - sep_axis * shA.radius;
            auto pivotB_world = closestB[i];
            point.pivotA = to_object_space(pivotA_world, posA, ornA);
            point.pivotB = to_object_space(pivotB_world, posB, ornB);
            result.add_point(point);
        }
    } else if (featureA == cylinder_feature::side_edge && featureB == box_feature::vertex) {
        point.pivotB = shB.get_vertex(feature_indexB);
        auto pivotB_world = to_world_space(point.pivotB, posB, ornB);
        vector3 closest; scalar t;
        closest_point_segment(cyl_vertices[0], cyl_vertices[1], pivotB_world, t, closest);

        auto pivotA_world = closest - sep_axis * shA.radius;
        point.pivotA = to_object_space(pivotA_world, posA, ornA);
        point.normal_attachment = contact_normal_attachment::none;
        result.add_point(point);
    } else if (featureA == cylinder_feature::cap_edge) {
        auto supportA = shA.support_point(posA, ornA, -sep_axis);
        point.pivotA = to_object_space(supportA, posA, ornA);
        point.pivotB = to_object_space(supportA - sep_axis * distance, posB, ornB);
        point.normal_attachment = featureB == box_feature::face ?
            contact_normal_attachment::normal_on_B :
            contact_normal_attachment::none;
        result.maybe_add_point(point);
    }
}

void collide(const box_shape &shA, const cylinder_shape &shB,
             const collision_context &ctx, collision_result &result) {
    swap_collide(shA, shB, ctx, result);
}

}
