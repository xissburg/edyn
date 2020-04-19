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
        axis.minB = -shB.half_extents[i];
        axis.maxB = shB.half_extents[i];
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
            axis.dir = normalize(cross(axisA, axisB));
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

    for (uint8_t i = 0; i < sep_axes.size(); ++i) {
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

    if (sep_axis.featureA == BOX_FEATURE_FACE && sep_axis.featureB == BOX_FEATURE_FACE) {
        // Face-Face.
        
    } else if (sep_axis.featureA == BOX_FEATURE_FACE && sep_axis.featureB == BOX_FEATURE_EDGE) {
        // Face A, Edge B.
        
    } else if (sep_axis.featureB == BOX_FEATURE_FACE && sep_axis.featureA == BOX_FEATURE_EDGE) {
        // Face B, Edge A.
        
    } else if (sep_axis.featureA == BOX_FEATURE_EDGE && sep_axis.featureB == BOX_FEATURE_EDGE) {
        // Edge-Edge.
        scalar s[2], t[2];
        vector3 p0[2], p1[2];
        size_t num_points = 0;
        auto [a0, a1] = shA.get_edge(sep_axis.feature_indexA);
        auto [b0, b1] = shB.get_edge(sep_axis.feature_indexB);
        a0 = posA + rotate(ornA, a0);
        a1 = posA + rotate(ornA, a1);
        b0 = posB + rotate(ornB, b0);
        b1 = posB + rotate(ornB, b1);
        closest_point_segment_segment(a0, a1, b0, b1, 
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
    } else if (sep_axis.featureA == BOX_FEATURE_FACE && sep_axis.featureB == BOX_FEATURE_VERTEX) {
        // Face A, Vertex B.
        auto normal = sep_axis.dir * (sep_axis.swap ? -1 : 1);
        auto pivotB = shB.get_vertex(sep_axis.feature_indexB);
        auto pivotA = (posB + rotate(ornB, pivotB)) - normal * sep_axis.distance;
        result.num_points = 1;
        result.point[0].pivotA = to_object_space(pivotA, posA, ornA);
        result.point[0].pivotB = pivotB;
        result.point[0].normalB = rotate(conjugate(ornB), normal);
        result.point[0].distance = sep_axis.distance;
    } else if (sep_axis.featureB == BOX_FEATURE_FACE && sep_axis.featureA == BOX_FEATURE_VERTEX) {
        // Face B, Vertex A.
        auto normal = sep_axis.dir * (sep_axis.swap ? -1 : 1);
        auto pivotA = shA.get_vertex(sep_axis.feature_indexA);
        auto pivotB = (posA + rotate(ornA, pivotA)) - normal * sep_axis.distance;
        result.num_points = 1;
        result.point[0].pivotA = pivotA;
        result.point[0].pivotB = to_object_space(pivotB, posB, ornB);
        result.point[0].normalB = rotate(conjugate(ornB), normal);
        result.point[0].distance = sep_axis.distance;
    }

    return result;
}

}