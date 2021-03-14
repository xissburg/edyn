#include "edyn/collision/collide.hpp"
#include <algorithm>
#include <numeric>
#include "edyn/shapes/box_shape.hpp"
#include "edyn/util/array.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/math/math.hpp"

namespace edyn {

struct box_box_separating_axis {
    vector3 dir;
    box_feature featureA;
    box_feature featureB;
    size_t feature_indexA;
    size_t feature_indexB;
    scalar distance;
};

collision_result collide(const box_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const box_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold) {
    // Box-Box SAT. Normal of 3 faces of A, normal of 3 faces of B, 3 * 3 edge
    // cross-products. Find axis with greatest projection.

    auto axesA = std::array<vector3, 3>{
        quaternion_x(ornA),
        quaternion_y(ornA),
        quaternion_z(ornA)
    };

    auto axesB = std::array<vector3, 3>{
        quaternion_x(ornB),
        quaternion_y(ornB),
        quaternion_z(ornB)
    };

    auto greatest_proj = -EDYN_SCALAR_MAX;
    size_t sep_axis_idx;

    // A's faces.
    for (size_t i = 0; i < 3; ++i) {
        auto dir = axesA[i];
        if (dot(posB - posA, dir) > 0) {
            dir = -dir; // Point towards A.
        }

        auto p = shB.support_point(posB, ornB, dir);
        auto proj = -(dot(dir, p - posA) + shA.half_extents[i]);

        if (proj > greatest_proj) {
            greatest_proj = proj;
            sep_axis_idx = i;
        }
    }

    // B's faces.
    for (size_t i = 0; i < 3; ++i) {
        auto dir = axesB[i];
        if (dot(posA - posB, dir) > 0) {
            dir = -dir; // Point towards B.
        }

        auto p = shA.support_point(posA, ornA, dir);
        auto proj = -(dot(dir, p - posB) + shB.half_extents[i]);

        if (proj > greatest_proj) {
            greatest_proj = proj;
            sep_axis_idx = 3 + i;
        }
    }

    // Edge-edge.
    for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < 3; ++j) {
            auto dir = cross(axesA[i], axesB[j]);
            auto dir_len_sqr = length_sqr(dir);

            if (dir_len_sqr <= EDYN_EPSILON) {
                continue;
            }

            dir /= std::sqrt(dir_len_sqr);

            if (dot(posA - posB, dir) < 0) {
                // Make it point towards A.
                dir *= -1;
            }
            
            auto pA = shA.support_point(posA, ornA, -dir);
            auto pB = shB.support_point(posB, ornB, dir);
            auto projA = dot(pA - posA, -dir);
            auto projB = dot(pB - posA, dir);
            auto proj = -(projA + projB);

            if (proj > greatest_proj) {
                greatest_proj = proj;
                sep_axis_idx = 6 + i * 3 + j;
            }
        }
    }

    if (greatest_proj > threshold) {
        return {};
    }

    box_box_separating_axis sep_axis;
    sep_axis.distance = greatest_proj;

    // Obtain support features for the chosen separating axis.
    if (sep_axis_idx < 3) {
        // A's faces.
        auto i = sep_axis_idx;
        auto &axisA = axesA[i];
        sep_axis.featureA = box_feature::face;

        if (dot(posB - posA, axisA) > 0) {
            sep_axis.feature_indexA = i * 2; // Positive face along axis.
            sep_axis.dir = -axisA; // Point towards A.
        } else {
            sep_axis.feature_indexA = i * 2 + 1; // Negative face along axis.
            sep_axis.dir = axisA; // Point towards A.
        }

        scalar distance;
        shB.support_feature(posB, ornB, posA, sep_axis.dir, 
                            sep_axis.featureB, sep_axis.feature_indexB, 
                            distance, threshold);
    } else if (sep_axis_idx < 6) {
        // B's faces.
        auto i = sep_axis_idx - 3;
        auto &axisB = axesB[i];
        sep_axis.featureB = box_feature::face;

        if (dot(posA - posB, axisB) > 0) {
            sep_axis.feature_indexB = i * 2; // Positive face along axis.
            sep_axis.dir = axisB; // Point towards A.
        } else {
            sep_axis.feature_indexB = i * 2 + 1; // Negative face along axis.
            sep_axis.dir = -axisB; // Point towards A.
        }

        scalar distance;
        shA.support_feature(posA, ornA, posB, -sep_axis.dir, 
                            sep_axis.featureA, sep_axis.feature_indexA, 
                            distance, threshold);
    } else {
        // Edge-edge.
        auto i = (sep_axis_idx - 6) / 3;
        auto j = (sep_axis_idx - 6) % 3;
        auto &axisA = axesA[i];
        auto &axisB = axesB[j];
        sep_axis.dir = normalize(cross(axisA, axisB));

        if (dot(posA - posB, sep_axis.dir) < 0) {
            // Make it point towards A.
            sep_axis.dir *= -1;
        }

        scalar projA, projB;
        shA.support_feature(posA, ornA, posB, -sep_axis.dir, 
                            sep_axis.featureA, sep_axis.feature_indexA, 
                            projA, threshold);
        shB.support_feature(posB, ornB, posB, sep_axis.dir, 
                            sep_axis.featureB, sep_axis.feature_indexB, 
                            projB, threshold);
    }

    auto result = collision_result{};
    auto normalB = rotate(conjugate(ornB), sep_axis.dir);

    if (sep_axis.featureA == box_feature::face && sep_axis.featureB == box_feature::face) {
        // Face-Face.
        auto face_verticesA = shA.get_face(sep_axis.feature_indexA, posA, ornA);
        auto face_normalA = shA.get_face_normal(sep_axis.feature_indexA, ornA);
        std::array<vector3, 4> face_tangentsA;
        for (size_t i = 0; i < 4; ++i) {
            auto &v0 = face_verticesA[i];
            auto &v1 = face_verticesA[(i + 1) % 4];
            face_tangentsA[i] = cross(face_normalA, v1 - v0);
        }

        auto face_verticesB = shB.get_face(sep_axis.feature_indexB, posB, ornB);
        auto face_normalB = shB.get_face_normal(sep_axis.feature_indexB, ornB);
        std::array<vector3, 4> face_tangentsB;
        for (size_t i = 0; i < 4; ++i) {
            auto &v0 = face_verticesB[i];
            auto &v1 = face_verticesB[(i + 1) % 4];
            face_tangentsB[i] = cross(face_normalB, v1 - v0);
        }

        // Check for vertices of Face B contained in Face A.
        for (size_t i = 0; i < 4; ++i) {
            if (result.num_points == max_contacts) {
                break;
            }

            scalar dots[4];
            for (size_t j = 0; j < 4; ++j) {
                dots[j] = dot(face_verticesB[i] - face_verticesA[j], face_tangentsA[j]);
            } 
            
            if (dots[0] > -EDYN_EPSILON && dots[1] > -EDYN_EPSILON && 
                dots[2] > -EDYN_EPSILON && dots[3] > -EDYN_EPSILON) {
                // Face B vertex is inside Face A.
                auto pivot_face = project_plane(face_verticesB[i], face_verticesA[0], face_normalA);
                auto pivotA = to_object_space(pivot_face, posA, ornA);
                auto pivotB = to_object_space(face_verticesB[i], posB, ornB);
                result.add_point({pivotA, pivotB, normalB, sep_axis.distance});
            }
        }

        // Check for vertices of Face A contained in Face B.
        for (size_t i = 0; i < 4; ++i) {
            if (result.num_points == max_contacts) {
                break;
            }

            scalar dots[4];
            for (size_t j = 0; j < 4; ++j) {
                dots[j] = dot(face_verticesA[i] - face_verticesB[j], face_tangentsB[j]);
            }

            if (dots[0] > -EDYN_EPSILON && dots[1] > -EDYN_EPSILON && 
                dots[2] > -EDYN_EPSILON && dots[3] > -EDYN_EPSILON) {
                // Face A vertex is inside Face B.
                auto pivot_face = project_plane(face_verticesA[i], face_verticesB[0], face_normalB);
                auto pivotA = to_object_space(face_verticesA[i], posA, ornA);
                auto pivotB = to_object_space(pivot_face, posB, ornB);
                result.add_point({pivotA, pivotB, normalB, sep_axis.distance});
            }
        }

        // If not all vertices are contained in a face, perform edge intersection tests.
        if (result.num_points < 4) {
            auto face_center = shA.get_face_center(sep_axis.feature_indexA, posA, ornA);
            auto face_basis = shA.get_face_basis(sep_axis.feature_indexA, ornA);
            auto half_extents = shA.get_face_half_extents(sep_axis.feature_indexA);

            for (size_t j = 0; j < 4; ++j) {
                auto b0_world = face_verticesB[j];
                auto b1_world = face_verticesB[(j + 1) % 4];
                auto b0 = to_object_space(b0_world, face_center, face_basis);
                auto b1 = to_object_space(b1_world, face_center, face_basis);
                auto p0 = vector2{b0.x, b0.z};
                auto p1 = vector2{b1.x, b1.z};

                scalar s[2];
                auto num_points = intersect_line_aabb(p0, p1, -half_extents, half_extents, s[0], s[1]);
                for (size_t k = 0; k < num_points; ++k) {
                    if (result.num_points == max_contacts) break;

                    if (s[k] >= 0 && s[k] <= 1) {
                        auto q1 = lerp(b0_world, b1_world, s[k]);
                        auto q0 = project_plane(q1, face_center, face_normalA);
                        auto pivotA = to_object_space(q0, posA, ornA);
                        auto pivotB = to_object_space(q1, posB, ornB);
                        result.add_point({pivotA, pivotB, normalB, sep_axis.distance});
                    }
                }

                if (result.num_points == max_contacts) break;
            }
        }
    } else if ((sep_axis.featureA == box_feature::face && sep_axis.featureB == box_feature::edge) ||
               (sep_axis.featureB == box_feature::face && sep_axis.featureA == box_feature::edge)) {
        // Face vs Edge.
        auto is_faceA = sep_axis.featureA == box_feature::face;

        auto face_normal = is_faceA ? shA.get_face_normal(sep_axis.feature_indexA, ornA) :
                                      shB.get_face_normal(sep_axis.feature_indexB, ornB);
        auto face_vertices = is_faceA ? shA.get_face(sep_axis.feature_indexA, posA, ornA) :
                                        shB.get_face(sep_axis.feature_indexB, posB, ornB);
        auto edge_vertices = is_faceA ? shB.get_edge(sep_axis.feature_indexB, posB, ornB) : 
                                        shA.get_edge(sep_axis.feature_indexA, posA, ornA);

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
                auto pivotA = is_faceA ? to_object_space(pivot_face, posA, ornA) :
                                         to_object_space(edge_vertices[i], posA, ornA);
                auto pivotB = is_faceA ? to_object_space(edge_vertices[i], posB, ornB) :
                                         to_object_space(pivot_face, posB, ornB);
                result.add_point({pivotA, pivotB, normalB, sep_axis.distance});
            }
        }

        // If both vertices are not inside the face then perform edge intersection tests.
        if (result.num_points < 2) {
            auto face_center = is_faceA ? shA.get_face_center(sep_axis.feature_indexA, posA, ornA) :
                                          shB.get_face_center(sep_axis.feature_indexB, posB, ornB);
            auto face_basis = is_faceA ? shA.get_face_basis(sep_axis.feature_indexA, ornA) :
                                         shB.get_face_basis(sep_axis.feature_indexB, ornB);
            auto half_extents = is_faceA ? shA.get_face_half_extents(sep_axis.feature_indexA) :
                                           shB.get_face_half_extents(sep_axis.feature_indexB);

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
                    auto pivotA = to_object_space(is_faceA ? face_pivot : edge_pivot, posA, ornA);
                    auto pivotB = to_object_space(is_faceA ? edge_pivot : face_pivot, posB, ornB);
                    result.add_point({pivotA, pivotB, normalB, sep_axis.distance});
                }
            }
        }
    } else if (sep_axis.featureA == box_feature::edge && sep_axis.featureB == box_feature::edge) {
        // Edge-Edge.
        scalar s[2], t[2];
        vector3 p0[2], p1[2];
        size_t num_points = 0;
        auto edgeA = shA.get_edge(sep_axis.feature_indexA, posA, ornA);
        auto edgeB = shB.get_edge(sep_axis.feature_indexB, posB, ornB);
        closest_point_segment_segment(edgeA[0], edgeA[1], edgeB[0], edgeB[1], 
                                      s[0], t[0], p0[0], p1[0], &num_points, 
                                      &s[1], &t[1], &p0[1], &p1[1]);

        for (size_t i = 0; i < num_points; ++i) {
            if (s[i] > 0 && s[i] < 1 && t[i] > 0 && t[i] < 1) {
                auto pivotA = to_object_space(p0[i], posA, ornA);
                auto pivotB = to_object_space(p1[i], posB, ornB);
                result.add_point({pivotA, pivotB, normalB, sep_axis.distance});
            }
        }
    } else if (sep_axis.featureA == box_feature::face && sep_axis.featureB == box_feature::vertex) {
        // Face A, Vertex B.
        auto pivotB = shB.get_vertex(sep_axis.feature_indexB);
        auto pivotA = (posB + rotate(ornB, pivotB)) + sep_axis.dir * sep_axis.distance;
        pivotA = to_object_space(pivotA, posA, ornA);
        result.add_point({pivotA, pivotB, normalB, sep_axis.distance});
    } else if (sep_axis.featureB == box_feature::face && sep_axis.featureA == box_feature::vertex) {
        // Face B, Vertex A.
        auto pivotA = shA.get_vertex(sep_axis.feature_indexA);
        auto pivotB = (posA + rotate(ornA, pivotA)) - sep_axis.dir * sep_axis.distance;
        pivotB = to_object_space(pivotB, posB, ornB);
        result.add_point({pivotA, pivotB, normalB, sep_axis.distance});
    }

    return result;
}

}