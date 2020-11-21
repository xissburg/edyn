#include "edyn/collision/collide.hpp"
#include <algorithm>
#include <numeric>
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
    // cross-products.
    std::array<box_box_separating_axis, 3 + 3 + 3 * 3> sep_axes;

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

    size_t axis_idx = 0;

    // A's faces.
    for (size_t i = 0; i < 3; ++i) {
        auto &axisA = axesA[i];
        auto &axis = sep_axes[axis_idx++];
        axis.featureA = BOX_FEATURE_FACE;

        if (dot(posB - posA, axisA) > 0) {
            axis.feature_indexA = i * 2; // Positive face along axis.
            axis.dir = -axisA; // Point towards A.
        } else {
            axis.feature_indexA = i * 2 + 1; // Negative face along axis.
            axis.dir = axisA; // Point towards A.
        }

        shB.support_feature(posB, ornB, posA, axis.dir, 
                            axis.featureB, axis.feature_indexB, 
                            axis.distance, threshold);
        // `axis.distance` contains the projection of the furthest feature with
        // respect to the center of A, thus it's necessary to add half the extent
        // of A to push it to the surface.
        // It also has to be negated because the projection given by `support_feature`
        // is in the direction `axis.dir` which points towards A and thus positive
        // projection has to be turned into penetration, which is intepreted as
        // negative distance.
        axis.distance = -(shA.half_extents[i] + axis.distance);
    }

    // B's faces.
    for (size_t i = 0; i < 3; ++i) {
        auto &axisB = axesB[i];
        auto &axis = sep_axes[axis_idx++];
        axis.featureB = BOX_FEATURE_FACE;

        if (dot(posA - posB, axisB) > 0) {
            axis.feature_indexB = i * 2; // Positive face along axis.
            axis.dir = axisB; // Point towards A.
        } else {
            axis.feature_indexB = i * 2 + 1; // Negative face along axis.
            axis.dir = -axisB; // Point towards A.
        }

        shA.support_feature(posA, ornA, posB, -axis.dir, 
                            axis.featureA, axis.feature_indexA, 
                            axis.distance, threshold);
        axis.distance = -(shB.half_extents[i] + axis.distance);
    }

    // Edge-edge.
    for (size_t i = 0; i < 3; ++i) {
        auto &axisA = axesA[i];

        for (size_t j = 0; j < 3; ++j) {
            auto &axisB = axesB[j];
            auto &axis = sep_axes[axis_idx];
            axis.dir = cross(axisA, axisB);
            auto dir_len_sqr = length_sqr(axis.dir);

            if (dir_len_sqr <= EDYN_EPSILON) {
                continue;
            }

            axis.dir /= std::sqrt(dir_len_sqr);

            if (dot(posA - posB, axis.dir) < 0) {
                // Make it point towards A.
                axis.dir *= -1;
            }

            scalar projA, projB;
            shA.support_feature(posA, ornA, posB, -axis.dir, 
                                axis.featureA, axis.feature_indexA, 
                                projA, threshold);
            shB.support_feature(posB, ornB, posB, axis.dir, 
                                axis.featureB, axis.feature_indexB, 
                                projB, threshold);
            axis.distance = -(projA + projB);

            ++axis_idx;
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

    if (sep_axis.featureA == BOX_FEATURE_FACE && sep_axis.featureB == BOX_FEATURE_FACE) {
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

                    if (s[k] >= 0 && s[k] < 1) {
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
    } else if ((sep_axis.featureA == BOX_FEATURE_FACE && sep_axis.featureB == BOX_FEATURE_EDGE) ||
               (sep_axis.featureB == BOX_FEATURE_FACE && sep_axis.featureA == BOX_FEATURE_EDGE)) {
        // Face vs Edge.
        auto is_faceA = sep_axis.featureA == BOX_FEATURE_FACE;

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
            for (int i = 0; i < 4; ++i) {
                auto &v0 = face_vertices[i];
                auto &v1 = face_vertices[(i + 1) % 4];

                scalar s[2], t[2];
                vector3 p0[2], p1[2];
                size_t num_points = 0;
                closest_point_segment_segment(v0, v1, edge_vertices[0], edge_vertices[1], 
                                              s[0], t[0], p0[0], p1[0], &num_points, 
                                              &s[1], &t[1], &p0[1], &p1[1]);
                for (size_t j = 0; j < num_points; ++j) {
                    if (s[j] > 0 && s[j] < 1 && t[j] > 0 && t[j] < 1) {
                        auto pivotA = is_faceA ? to_object_space(p0[j], posA, ornA) :
                                                 to_object_space(p1[j], posA, ornA);
                        auto pivotB = is_faceA ? to_object_space(p1[j], posB, ornB) :
                                                 to_object_space(p0[j], posB, ornB);
                        result.add_point({pivotA, pivotB, normalB, sep_axis.distance});
                    }
                }
            }
        }        
    } else if (sep_axis.featureA == BOX_FEATURE_EDGE && sep_axis.featureB == BOX_FEATURE_EDGE) {
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
    } else if (sep_axis.featureA == BOX_FEATURE_FACE && sep_axis.featureB == BOX_FEATURE_VERTEX) {
        // Face A, Vertex B.
        auto pivotB = shB.get_vertex(sep_axis.feature_indexB);
        auto pivotA = (posB + rotate(ornB, pivotB)) + sep_axis.dir * sep_axis.distance;
        pivotA = to_object_space(pivotA, posA, ornA);
        result.add_point({pivotA, pivotB, normalB, sep_axis.distance});
    } else if (sep_axis.featureB == BOX_FEATURE_FACE && sep_axis.featureA == BOX_FEATURE_VERTEX) {
        // Face B, Vertex A.
        auto pivotA = shA.get_vertex(sep_axis.feature_indexA);
        auto pivotB = (posA + rotate(ornA, pivotA)) - sep_axis.dir * sep_axis.distance;
        pivotB = to_object_space(pivotB, posB, ornB);
        result.add_point({pivotA, pivotB, normalB, sep_axis.distance});
    }

    return result;
}

}