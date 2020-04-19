#include "edyn/collision/collide.hpp"
#include <algorithm>
#include <numeric>

namespace edyn {

struct box_box_separating_axis {
    vector3 dir;
    scalar minA, maxA;
    scalar minB, maxB;
    bool swap {false};
    box_feature min_featureA;
    box_feature max_featureB;
    box_feature min_featureB;
    box_feature max_featureA;
    uint8_t min_feature_indexA;
    uint8_t max_feature_indexB;
    uint8_t min_feature_indexB;
    uint8_t max_feature_indexA;
    box_feature featureA;
    box_feature featureB;
    uint8_t feature_indexA;
    uint8_t feature_indexB;
    scalar distance;
};

collision_result collide(const box_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const box_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold) {
    // Box-Box SAT.
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

    uint8_t axis_idx = 0;

    for (uint8_t i = 0; i < 3; ++i) {
        auto &axisA = axesA[i];
        auto &axis = sep_axes[axis_idx];
        axis.dir = axisA;
        auto proj_posA = dot(posA, axisA);
        axis.minA = proj_posA - shA.half_extents[i];
        axis.maxA = proj_posA + shA.half_extents[i];
        axis.min_featureA = BOX_FEATURE_FACE;
        axis.max_featureA = BOX_FEATURE_FACE;
        axis.min_feature_indexA = i * 2 + 1;
        axis.max_feature_indexA = i * 2;

        auto [min_featureB, min_feature_indexB] = shB.support_feature(ornB, -axis.dir);
        axis.min_featureB = min_featureB;
        axis.min_feature_indexB = min_feature_indexB;
        axis.minB = dot(axis.dir, shB.support_point(posB, ornB, -axis.dir));

        auto [max_featureB, max_feature_indexB] = shB.support_feature(ornB, axis.dir);
        axis.max_featureB = max_featureB;
        axis.max_feature_indexB = max_feature_indexB;
        axis.maxB = dot(axis.dir, shB.support_point(posB, ornB, axis.dir));

        ++axis_idx;
    }

    for (uint8_t i = 0; i < 3; ++i) {
        auto &axisB = axesB[i];
        auto &axis = sep_axes[axis_idx];
        axis.dir = axisB;
        auto proj_posB = dot(posB, axisB);
        axis.minB = proj_posB - shB.half_extents[i];
        axis.maxB = proj_posB + shB.half_extents[i];
        axis.min_featureB = BOX_FEATURE_FACE;
        axis.max_featureB = BOX_FEATURE_FACE;
        axis.min_feature_indexB = i * 2 + 1;
        axis.max_feature_indexB = i * 2;

        auto [min_featureA, min_feature_indexA] = shA.support_feature(ornA, -axis.dir);
        axis.min_featureA = min_featureA;
        axis.min_feature_indexA = min_feature_indexA;
        axis.minA = dot(axis.dir, shA.support_point(posA, ornA, -axis.dir));

        auto [max_featureA, max_feature_indexA] = shA.support_feature(ornA, axis.dir);
        axis.max_featureA = max_featureA;
        axis.max_feature_indexA = max_feature_indexA;
        axis.maxA = dot(axis.dir, shA.support_point(posA, ornA, axis.dir));

        ++axis_idx;
    }

    for (uint8_t i = 0; i < 3; ++i) {
        auto &axisA = axesA[i];

        for (uint8_t j = 0; j < 3; ++j) {
            auto &axisB = axesB[j];
            auto &axis = sep_axes[axis_idx];
            axis.dir = cross(axisA, axisB);
            auto dir_len_sqr = length2(axis.dir);

            if (dir_len_sqr <= EDYN_EPSILON) {
                continue;
            }

            axis.dir /= std::sqrt(dir_len_sqr);
            axis.minA = dot(axis.dir, shA.support_point(posA, ornA, -axis.dir));
            axis.maxA = dot(axis.dir, shA.support_point(posA, ornA, axis.dir));
            axis.minB = dot(axis.dir, shB.support_point(posB, ornB, -axis.dir));
            axis.maxB = dot(axis.dir, shB.support_point(posB, ornB, axis.dir));
        
            auto [min_featureA, min_feature_indexA] = shA.support_feature(ornA, -axis.dir);
            axis.min_featureA = min_featureA;
            axis.min_feature_indexA = min_feature_indexA;
        
            auto [max_featureA, max_feature_indexA] = shA.support_feature(ornA, axis.dir);
            axis.max_featureA = max_featureA;
            axis.max_feature_indexA = max_feature_indexA;
        
            auto [min_featureB, min_feature_indexB] = shB.support_feature(ornB, -axis.dir);
            axis.min_featureB = min_featureB;
            axis.min_feature_indexB = min_feature_indexB;

            auto [max_featureB, max_feature_indexB] = shB.support_feature(ornB, axis.dir);
            axis.max_featureB = max_featureB;
            axis.max_feature_indexB = max_feature_indexB;

            ++axis_idx;
        }
    }

    auto greatest_distance = -EDYN_SCALAR_MAX;
    uint8_t sep_axis_idx;

    for (uint8_t i = 0; i < axis_idx; ++i) {
        auto &sep_axis = sep_axes[i];

        if (sep_axis.maxA < sep_axis.minB) {
            // B's interval is in front of A's.
            sep_axis.distance = sep_axis.minB - sep_axis.maxA;
            // Choose max feature for A.
            sep_axis.featureA = sep_axis.max_featureA;
            sep_axis.feature_indexA = sep_axis.max_feature_indexA;
            // Choose min feature for B.
            sep_axis.featureB = sep_axis.min_featureB;
            sep_axis.feature_indexB = sep_axis.min_feature_indexB;
            // Axis must be flipped to point towards A.
            sep_axis.swap = true;
        } else if (sep_axis.maxB < sep_axis.minA) {
            // A's interval is in front of B's.
            sep_axis.distance = sep_axis.minA - sep_axis.maxB;
            // Choose min feature for A.
            sep_axis.featureA = sep_axis.min_featureA;
            sep_axis.feature_indexA = sep_axis.min_feature_indexA;
            // Choose max feature for B.
            sep_axis.featureB = sep_axis.max_featureB;
            sep_axis.feature_indexB = sep_axis.max_feature_indexB;
        }

        if (sep_axis.minB < sep_axis.maxA) {
            if (sep_axis.maxB < sep_axis.maxA) {
                if (sep_axis.maxA - sep_axis.minB < sep_axis.maxB - sep_axis.minA) {
                    sep_axis.distance = sep_axis.minB - sep_axis.maxA;
                    // Choose max feature for A.
                    sep_axis.featureA = sep_axis.max_featureA;
                    sep_axis.feature_indexA = sep_axis.max_feature_indexA;
                    // Choose min feature for B.
                    sep_axis.featureB = sep_axis.min_featureB;
                    sep_axis.feature_indexB = sep_axis.min_feature_indexB;
                    sep_axis.swap = true;
                } else {
                    sep_axis.distance = sep_axis.minA - sep_axis.maxB;
                    // Choose min feature for A.
                    sep_axis.featureA = sep_axis.min_featureA;
                    sep_axis.feature_indexA = sep_axis.min_feature_indexA;
                    // Choose max feature for B.
                    sep_axis.featureB = sep_axis.max_featureB;
                    sep_axis.feature_indexB = sep_axis.max_feature_indexB;
                }
            } else {
                sep_axis.distance = sep_axis.minB - sep_axis.maxA;
                // Choose max feature for A.
                sep_axis.featureA = sep_axis.max_featureA;
                sep_axis.feature_indexA = sep_axis.max_feature_indexA;
                // Choose min feature for B.
                sep_axis.featureB = sep_axis.min_featureB;
                sep_axis.feature_indexB = sep_axis.min_feature_indexB;
                sep_axis.swap = true;
            }
        } else {
            if (sep_axis.minA < sep_axis.minB) {
                if (sep_axis.maxB - sep_axis.minA < sep_axis.maxA - sep_axis.minB) {
                    sep_axis.distance = sep_axis.minA - sep_axis.maxB;
                    // Choose min feature for A.
                    sep_axis.featureA = sep_axis.min_featureA;
                    sep_axis.feature_indexA = sep_axis.min_feature_indexA;
                    // Choose max feature for B.
                    sep_axis.featureB = sep_axis.max_featureB;
                    sep_axis.feature_indexB = sep_axis.max_feature_indexB;
                } else {
                    sep_axis.distance = sep_axis.minB - sep_axis.maxA;
                    // Choose max feature for A.
                    sep_axis.featureA = sep_axis.max_featureA;
                    sep_axis.feature_indexA = sep_axis.max_feature_indexA;
                    // Choose min feature for B.
                    sep_axis.featureB = sep_axis.min_featureB;
                    sep_axis.feature_indexB = sep_axis.min_feature_indexB;
                    sep_axis.swap = true;
                }
            } else {
                sep_axis.distance = sep_axis.minA - sep_axis.maxB;
                // Choose min feature for A.
                sep_axis.featureA = sep_axis.min_featureA;
                sep_axis.feature_indexA = sep_axis.min_feature_indexA;
                // Choose max feature for B.
                sep_axis.featureB = sep_axis.max_featureB;
                sep_axis.feature_indexB = sep_axis.max_feature_indexB;
            }
        }
        
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
    auto normal_world = sep_axis.dir * (sep_axis.swap ? -1 : 1);
    auto normalB = rotate(conjugate(ornB), normal_world);

    if (sep_axis.featureA == BOX_FEATURE_FACE && sep_axis.featureB == BOX_FEATURE_FACE) {
        // Face-Face.
        // Intersect each edge of face A with face B.
        auto face_verticesA = shA.get_face(sep_axis.feature_indexA, posA, ornA);
        auto face_normalA = shA.get_face_normal(sep_axis.feature_indexA, ornA);
        std::array<vector3, 4> face_tangentsA;
        for (uint8_t i = 0; i < 4; ++i) {
            auto &v0 = face_verticesA[i];
            auto &v1 = face_verticesA[(i + 1) % 4];
            face_tangentsA[i] = cross(face_normalA, v1 - v0);
        }

        auto face_verticesB = shB.get_face(sep_axis.feature_indexB, posB, ornB);
        auto face_normalB = shB.get_face_normal(sep_axis.feature_indexB, ornB);
        std::array<vector3, 4> face_tangentsB;
        for (uint8_t i = 0; i < 4; ++i) {
            auto &v0 = face_verticesB[i];
            auto &v1 = face_verticesB[(i + 1) % 4];
            face_tangentsB[i] = cross(face_normalB, v1 - v0);
        }

        // Check for vertices of Face B contained in Face A.
        for (uint8_t i = 0; i < 4; ++i) {
            if (dot(face_verticesB[i] - face_verticesA[0], face_tangentsA[0]) > 0 &&
                dot(face_verticesB[i] - face_verticesA[1], face_tangentsA[1]) > 0 &&
                dot(face_verticesB[i] - face_verticesA[2], face_tangentsA[2]) > 0 &&
                dot(face_verticesB[i] - face_verticesA[3], face_tangentsA[3]) > 0) {
                // Face B vertex is inside Face A.
                auto pivot_face = project_plane(face_verticesB[i], face_verticesA[0], face_normalA);
                auto idx = result.num_points++;
                result.point[idx].pivotA = to_object_space(pivot_face, posA, ornA);
                result.point[idx].pivotB = to_object_space(face_verticesB[i], posB, ornB);
                result.point[idx].normalB = normalB;
                result.point[idx].distance = sep_axis.distance;
            }
        }

        // Check for vertices of Face A contained in Face B.
        for (uint8_t i = 0; i < 4; ++i) {
            if (dot(face_verticesA[i] - face_verticesB[0], face_tangentsB[0]) > 0 &&
                dot(face_verticesA[i] - face_verticesB[1], face_tangentsB[1]) > 0 &&
                dot(face_verticesA[i] - face_verticesB[2], face_tangentsB[2]) > 0 &&
                dot(face_verticesA[i] - face_verticesB[3], face_tangentsB[3]) > 0) {
                // Face A vertex is inside Face B.
                auto pivot_face = project_plane(face_verticesA[i], face_verticesB[0], face_normalB);
                auto idx = result.num_points++;
                result.point[idx].pivotA = to_object_space(face_verticesA[i], posA, ornA);
                result.point[idx].pivotB = to_object_space(pivot_face, posB, ornB);
                result.point[idx].normalB = normalB;
                result.point[idx].distance = sep_axis.distance;
            }
        }

        // If not all vertices are contained in a face, perform edge intersection tests.
        if (result.num_points < 4) {
            for (uint8_t i = 0; i < 4; ++i) {
                auto &a0 = face_verticesA[i];
                auto &a1 = face_verticesA[(i + 1) % 4];

                for (uint8_t j = 0; j < 4; ++j) {
                    auto &b0 = face_verticesB[j];
                    auto &b1 = face_verticesB[(j + 1) % 4];

                    scalar s[2], t[2];
                    vector3 p0[2], p1[2];
                    size_t num_points = 0;
                    closest_point_segment_segment(a0, a1, b0, b1, 
                                                s[0], t[0], p0[0], p1[0], &num_points, 
                                                &s[1], &t[1], &p0[1], &p1[1]);
                    for (uint8_t i = 0; i < num_points; ++i) {
                        if (s[i] > 0 && s[i] < 1 && t[i] > 0 && t[i] < 1) {
                            auto idx = result.num_points++;
                            result.point[idx].pivotA = to_object_space(p0[i], posA, ornA);
                            result.point[idx].pivotB = to_object_space(p1[i], posB, ornB);
                            result.point[idx].normalB = normalB;
                            result.point[idx].distance = sep_axis.distance;

                            if (result.num_points == max_contacts) break;
                        }
                    }

                    if (result.num_points == max_contacts) break;
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
        for (uint8_t i = 0; i < 4; ++i) {
            auto &v0 = face_vertices[i];
            auto &v1 = face_vertices[(i + 1) % 4];
            face_tangents[i] = cross(face_normal, v1 - v0);
        }

        for (uint8_t i = 0; i < 2; ++i) {
            if (dot(edge_vertices[i] - face_vertices[0], face_tangents[0]) > 0 &&
                dot(edge_vertices[i] - face_vertices[1], face_tangents[1]) > 0 &&
                dot(edge_vertices[i] - face_vertices[2], face_tangents[2]) > 0 &&
                dot(edge_vertices[i] - face_vertices[3], face_tangents[3]) > 0) {
                // Edge's vertex is inside face.
                auto pivot_face = project_plane(edge_vertices[i], face_vertices[0], face_normal);
                auto idx = result.num_points++;
                result.point[idx].pivotA = is_faceA ? to_object_space(pivot_face, posA, ornA) :
                                                      to_object_space(edge_vertices[i], posA, ornA);
                result.point[idx].pivotB = is_faceA ? to_object_space(edge_vertices[i], posB, ornB) :
                                                      to_object_space(pivot_face, posB, ornB);
                result.point[idx].normalB = normalB;
                result.point[idx].distance = sep_axis.distance;
            }
        }

        // If both vertices are not inside the face then perform edge intersection tests.
        if (result.num_points < 2) {
            for (uint8_t i = 0; i < 4; ++i) {
                auto &v0 = face_vertices[i];
                auto &v1 = face_vertices[(i + 1) % 4];

                scalar s[2], t[2];
                vector3 p0[2], p1[2];
                size_t num_points = 0;
                closest_point_segment_segment(v0, v1, edge_vertices[0], edge_vertices[1], 
                                              s[0], t[0], p0[0], p1[0], &num_points, 
                                              &s[1], &t[1], &p0[1], &p1[1]);
                for (uint8_t i = 0; i < num_points; ++i) {
                    if (s[i] > 0 && s[i] < 1 && t[i] > 0 && t[i] < 1) {
                        auto idx = result.num_points++;
                        result.point[idx].pivotA = is_faceA ? to_object_space(p0[i], posA, ornA) :
                                                              to_object_space(p1[i], posA, ornA);
                        result.point[idx].pivotB = is_faceA ? to_object_space(p1[i], posB, ornB) :
                                                              to_object_space(p0[i], posB, ornB);
                        result.point[idx].normalB = normalB;
                        result.point[idx].distance = sep_axis.distance;
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

        for (uint8_t i = 0; i < num_points; ++i) {
            if (s[i] > 0 && s[i] < 1 && t[i] > 0 && t[i] < 1) {
                auto idx = result.num_points++;
                result.point[idx].pivotA = to_object_space(p0[i], posA, ornA);
                result.point[idx].pivotB = to_object_space(p1[i], posB, ornB);
                result.point[idx].normalB = normalB;
                result.point[idx].distance = sep_axis.distance;
            }
        }
    } else if (sep_axis.featureA == BOX_FEATURE_FACE && sep_axis.featureB == BOX_FEATURE_VERTEX) {
        // Face A, Vertex B.
        auto pivotB = shB.get_vertex(sep_axis.feature_indexB);
        auto pivotA = (posB + rotate(ornB, pivotB)) - normal_world * sep_axis.distance;
        result.num_points = 1;
        result.point[0].pivotA = to_object_space(pivotA, posA, ornA);
        result.point[0].pivotB = pivotB;
        result.point[0].normalB = normalB;
        result.point[0].distance = sep_axis.distance;
    } else if (sep_axis.featureB == BOX_FEATURE_FACE && sep_axis.featureA == BOX_FEATURE_VERTEX) {
        // Face B, Vertex A.
        auto pivotA = shA.get_vertex(sep_axis.feature_indexA);
        auto pivotB = (posA + rotate(ornA, pivotA)) - normal_world * sep_axis.distance;
        result.num_points = 1;
        result.point[0].pivotA = pivotA;
        result.point[0].pivotB = to_object_space(pivotB, posB, ornB);
        result.point[0].normalB = normalB;
        result.point[0].distance = sep_axis.distance;
    }

    return result;
}

}