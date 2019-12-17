#include "edyn/collision/collide.hpp"
#include "edyn/math/geom.hpp"
#include <array>

namespace edyn {

collision_result collide(const cylinder_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const cylinder_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold) {
    // Half-length vector in world-space.
    const auto hlA = rotate(ornA, vector3_x * shA.half_length);
    const auto hlB = rotate(ornB, vector3_x * shB.half_length);

    // Center of discs on cylinder's ends.
    const auto p0A = posA - hlA;
    const auto p1A = posA + hlA;

    const auto p0B = posB - hlB;
    const auto p1B = posB + hlB;

    scalar s0, t0, s1, t1;
    vector3 cA0, cB0, cA1, cB1;

    size_t num_points;

    auto l2 = closest_point_segment_segment(p0A, p1A, p0B, p1B, s0, t0, 
                                            cA0, cB0, 
                                            &num_points, &s1, &t1, 
                                            &cA1, &cB1);

    // Minimum distance between segments to move forward, i.e. the cylinder
    // is bounded by a capsule.
    const auto min_dist = shA.radius + shB.radius + threshold;
    
    if (l2 > min_dist * min_dist) {
        return {};
    }

    auto dA = p0A - p1A;
    auto dB = p0B - p1B;
    auto d0 = p0A - p0B;
    auto d1 = p1A - p1B;
    auto d = cA0 - cB0;

    // Check whether the closest points between segments are on the vertices
    // using the angle between the segment connecting the closest points and
    // the A and B segments.
    auto ddA = dot(d, dA);
    auto ddB = dot(-d, dB);
    auto is_vertexA = std::abs(ddA) > EDYN_EPSILON;
    auto is_vertexB = std::abs(ddB) > EDYN_EPSILON;

    closest_points_array closest;
    vector3 normal;

    if (!is_vertexA && !is_vertexB) {
        // Closest points are on both cylindrical surfaces.
        normal = l2 > EDYN_EPSILON ? normal = d / std::sqrt(l2) : vector3_y;

        closest[0].first = cA0 - normal * shA.radius;
        closest[0].second = cB0 + normal * shB.radius;

        if (num_points > 1) {
            closest[1].first = cA1 - normal * shA.radius;
            closest[1].second = cB1 + normal * shB.radius;
        }
    } else {
        scalar sA0 = 0, sA1 = 0, sB0 = 0, sB1 = 0;
        scalar dist2A = EDYN_SCALAR_MAX, dist2B = EDYN_SCALAR_MAX;
        size_t num_pointsA = 0, num_pointsB = 0;
        vector3 normalA, normalB;
        vector3 cA0A, cA1A, cB0A, cB1A;
        vector3 cA0B, cA1B, cB0B, cB1B;
        scalar sideA = ddA > EDYN_EPSILON ? 1 : -1;
        scalar sideB = ddB > EDYN_EPSILON ? 1 : -1;

        if (is_vertexA) {
            // Closest point might be on one of A's disc and on B's segment.
            auto orn = ornA;
            if (sideA < 0) {
                orn *= quaternion_axis_angle(vector3_y, pi);
            }
            dist2A = closest_point_disc_line(posA + hlA * sideA, orn, shA.radius,
                                             p0B, p1B, num_pointsA, 
                                             sA0, cA0A, cB0A,
                                             sA1, cA1A, cB1A, normalA);
        }

        if (is_vertexB) {
            // Closest point might be on one of B's disc and on A's segment.
            auto orn = ornB;
            if (sideB < 0) {
                orn *= quaternion_axis_angle(vector3_y, pi);
            }
            dist2B = closest_point_disc_line(posB + hlB * sideB, orn, shB.radius,
                                             p0A, p1A, num_pointsB,
                                             sB0, cB0B, cA0B,
                                             sB1, cB1B, cA1B, normalB);
        }

        // If A's disc is closer to B's segment than B's disc is to A's segment,
        // and the parameters `sA0` or `sA1` are within the segment's range 
        // [0,1], then this should be the closest features. 
        // The same is done the other way around in the other if-statement.
        if (dist2A < dist2B && ((sA0 > 0 && sA0 < 1) || (num_pointsA > 1 && sA1 > 0 && sA1 < 1))) {
            if (dist2A > (threshold + shB.radius) * (threshold + shB.radius)) {
                return {};
            }

            num_points = num_pointsA;
            normal = -normalA;
            closest[0].first = cA0A;
            closest[0].second = cB0A + normal * shB.radius;

            if (num_points > 1) {
                closest[1].first = cA1A;
                closest[1].second = cB1A + normal * shB.radius;
            }
        } else if (dist2A >= dist2B && ((sB0 > 0 && sB0 < 1) || (num_pointsB > 1 && sB1 > 0 && sB1 < 1))) {
            if (dist2B > (threshold + shA.radius) * (threshold + shA.radius)) {
                return {};
            }

            num_points = num_pointsB;
            normal = normalB;
            closest[0].first = cA0B - normal * shA.radius;
            closest[0].second = cB0B;

            if (num_points > 1) {
                closest[1].first = cA1B - normal * shA.radius;
                closest[1].second = cB1B;
            }
        } else {
            auto ccl2 = closest_point_disc_disc(posA + hlA * sideA, ornA, shA.radius,
                                                posB + hlB * sideB, ornB, shB.radius,
                                                num_points, closest, normal);

            if (ccl2 > threshold * threshold) {
                return {};
            }
        }
    }

    auto result = collision_result {};
    result.num_points = num_points;

    for (size_t i = 0; i < num_points; ++i) {
        auto rA = closest[i].first - posA;
        auto rB = closest[i].second - posB;
        result.point[i].pivotA = rotate(conjugate(ornA), rA);
        result.point[i].pivotB = rotate(conjugate(ornB), rB);
        result.point[i].normalB = rotate(conjugate(ornB), normal);
        result.point[i].distance = dot(closest[i].first - closest[i].second, normal);
    }

    return result;
}

}