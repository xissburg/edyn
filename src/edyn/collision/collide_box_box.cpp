#include "edyn/collision/collide.hpp"

namespace edyn {

enum box_feature {
    BOX_FEATURE_VERTEX,
    BOX_FEATURE_EDGE,
    BOX_FEATURE_FACE
};

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
    scalar distance;
};

scalar interval_distance(scalar minA, scalar maxA, scalar minB, scalar maxB) {
    if (maxA < minB) {
        return minB - maxA;
    } else if (maxB < minA) {
        return minA - maxB;
    }

    if (minA < maxB) {
        if (maxB < maxA) {
            return std::max(minB - maxA, minA - maxB);
        } else {
            return minB - maxA;
        }
    } else {
        if (minA < minB) {
            return std::max(minA - maxB, minB - maxA);
        } else {
            return minA - maxB;
        }
    }
}

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
        axis.minA = -shA.half_extents[i];
        axis.maxA = shA.half_extents[i];
        axis.minB = dot(axis.dir, shB.support_point(posB, ornB, -axis.dir));
        axis.maxB = dot(axis.dir, shB.support_point(posB, ornB, axis.dir));

        axis.min_featureA = BOX_FEATURE_FACE;
        axis.max_featureA = BOX_FEATURE_FACE;
        axis.min_feature_indexA = i * 3;
        axis.max_feature_indexA = i;

        ++axis_idx;
    }

    for (uint8_t i = 0; i < 3; ++i) {
        auto &axisB = axesB[i];
        auto &axis = sep_axes[axis_idx];
        axis.dir = axisB;
        axis.minB = -shB.half_extents[i];
        axis.maxB = shB.half_extents[i];
        axis.minA = dot(axis.dir, shA.support_point(posA, ornA, -axis.dir));
        axis.maxA = dot(axis.dir, shA.support_point(posA, ornA, axis.dir));

        axis.min_featureB = BOX_FEATURE_FACE;
        axis.max_featureB = BOX_FEATURE_FACE;
        axis.min_feature_indexB = i * 3;
        axis.max_feature_indexB = i;

        ++axis_idx;
    }

    for (uint8_t i = 0; i < 3; ++i) {
        auto &axisA = axesA[i];

        for (uint8_t j = 0; j < 3; ++j) {
            auto &axisB = axesB[j];
            auto &axis = sep_axes[axis_idx];
            axis.dir = normalize(cross(axisA, axisB));
            axis.minA = dot(axis.dir, shA.support_point(posA, ornA, -axis.dir));
            axis.maxA = dot(axis.dir, shA.support_point(posA, ornA, axis.dir));
            axis.minB = dot(axis.dir, shB.support_point(posB, ornB, -axis.dir));
            axis.maxB = dot(axis.dir, shB.support_point(posB, ornB, axis.dir));

            ++axis_idx;
        }
    }

    auto greatest_distance = -EDYN_SCALAR_MAX;
    uint8_t sep_axis_idx;

    for (uint8_t i = 0; i < sep_axes.size(); ++i) {
        auto &sep_axis = sep_axes[i];

        if (sep_axis.maxA < sep_axis.minB) {
            sep_axis.distance = sep_axis.minB - sep_axis.maxA;
            sep_axis.swap = true;
        } else if (sep_axis.maxB < sep_axis.minA) {
            sep_axis.distance = sep_axis.minA - sep_axis.maxB;
        }

        if (sep_axis.minB < sep_axis.maxA) {
            if (sep_axis.maxB < sep_axis.maxA) {
                if (sep_axis.maxA - sep_axis.minB < sep_axis.maxB - sep_axis.minA) {
                    sep_axis.distance = sep_axis.minB - sep_axis.maxA;
                    sep_axis.swap = true;
                } else {
                    sep_axis.distance = sep_axis.minA - sep_axis.maxB;
                }
            } else {
                sep_axis.distance = sep_axis.minB - sep_axis.maxA;
                sep_axis.swap = true;
            }
        } else {
            if (sep_axis.minA < sep_axis.minB) {
                if (sep_axis.maxB - sep_axis.minA < sep_axis.maxA - sep_axis.minB) {
                    sep_axis.distance = sep_axis.minA - sep_axis.maxB;
                } else {
                    sep_axis.distance = sep_axis.minB - sep_axis.maxA;
                    sep_axis.swap = true;
                }
            } else {
                sep_axis.distance = sep_axis.minA - sep_axis.maxB;
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
    std::vector<vector3> verticesA, verticesB;
    
    for (auto &v : shA.vertices(posA, ornA)) {
        auto proj = dot(v, sep_axis.dir);
        if (std::abs(proj - (sep_axis.swap ? sep_axis.maxA : sep_axis.minA)) < threshold) {
            verticesA.push_back(v);
        }
    }
    
    for (auto &v : shB.vertices(posB, ornB)) {
        auto proj = dot(v, sep_axis.dir);
        if (std::abs(proj - (sep_axis.swap ? sep_axis.minB : sep_axis.maxB)) < threshold) {
            verticesB.push_back(v);
        }
    }

    if (verticesA.size() == 4 && verticesB.size() == 4) {
        // Face-Face.

    } else if (verticesA.size() == 4 && verticesB.size() >= 2) {
        // Face A, Edge B.

    } else if (verticesB.size() == 4 && verticesA.size() >= 2) {
        // Face B, Edge A.

    } else if (verticesA.size() >= 2 && verticesB.size() >= 2) {
        // Edge-Edge.
        scalar s[2], t[2];
        vector3 p0[2], p1[2];
        size_t num_points = 0;
        closest_point_segment_segment(verticesA[0], verticesA[1], 
                                      verticesB[0], verticesB[1],
                                      s[0], t[0], p0[0], p1[0], &num_points, 
                                      &s[1], &t[1], &p0[1], &p1[1]);

        for (uint8_t i = 0; i < num_points; ++i) {
            if (s[i] > 0 && s[i] < 1 && t[i] > 0 && t[i] < 1) {
                auto idx = result.num_points++;
                result.point[idx].pivotA = to_object_space(p0[i], posA, ornA);
                result.point[idx].pivotB = to_object_space(p1[i], posB, ornB);
                result.point[idx].normalB = sep_axis.dir * (sep_axis.swap ? -1 : 1) ;
                result.point[idx].distance = sep_axis.distance;
            }
        }
    } else if (verticesA.size() == 4) {
        // Face A, Vertex B.
        auto normal = sep_axis.dir * (sep_axis.swap ? -1 : 1);
        auto pivotB = verticesB[0];
        auto pivotA = pivotB - normal * sep_axis.distance;
        result.num_points = 1;
        result.point[0].pivotA = to_object_space(pivotA, posA, ornA);
        result.point[0].pivotB = to_object_space(pivotB, posB, ornB);
        result.point[0].normalB = rotate(conjugate(ornB), normal);
        result.point[0].distance = sep_axis.distance;
    } else if (verticesB.size() == 4) {
        // Face B, Vertex A.
        auto normal = sep_axis.dir * (sep_axis.swap ? -1 : 1);
        auto pivotA = verticesA[0];
        auto pivotB = pivotA - normal * sep_axis.distance;
        result.num_points = 1;
        result.point[0].pivotA = to_object_space(pivotA, posA, ornA);
        result.point[0].pivotB = to_object_space(pivotB, posB, ornB);
        result.point[0].normalB = rotate(conjugate(ornB), normal);
        result.point[0].distance = sep_axis.distance;
    }

    return result;
}

}