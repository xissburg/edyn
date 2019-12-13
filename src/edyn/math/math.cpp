#include "edyn/math/math.hpp"
#include <algorithm>

namespace edyn {

scalar closest_point_segment(const vector3 &q0, const vector3 &q1,
                             const vector3 &p, scalar &t, vector3 &q) {
    auto v = q1 - q0; // Direction vector of segment `q`.
    auto w = p - q0; // Vector from initial point of segment to point `p`.
    auto a = dot(w, v);
    auto b = dot(v, v);
    EDYN_ASSERT(b > EDYN_EPSILON);
    t = std::clamp(a / b, scalar(0), scalar(1));
    q = q0 + v * t;
    return length2(p - q);
}

// Reference: Real-Time Collision Detection - Christer Ericson, section 5.1.9.
scalar closest_point_segment_segment(const vector3 &p1, const vector3 &q1, 
                                     const vector3 &p2, const vector3 &q2, 
                                     scalar &s, scalar &t, 
                                     vector3 &c1, vector3 &c2) {
    const auto d1 = q1 - p1; // Direction vector of segment `s1`.
    const auto d2 = q2 - p2; // Direction vector of segment `s2`.
    const auto r = p1 - p2;
    const auto a = dot(d1, d1); // Squared length of segment `s1`.
    const auto e = dot(d2, d2); // Squared length of segment `s2`.
    const auto f = dot(d2, r);

    // Check if either or both segments degenerate into points.
    if (a <= EDYN_EPSILON && e <= EDYN_EPSILON) {
        // Both segments degenerate into points.
        s = t = 0;
        c1 = p1;
        c2 = p2;
        return length2(c1 - c2);
    }

    if (a <= EDYN_EPSILON) {
        // `s1` degenerates into a point.
        s = 0;
        t = f / e; // s = 0 => t = (b * s + f) / e = f / e;
        t = std::clamp(t, scalar(0), scalar(1));
    } else {
        auto c = dot(d1, r);

        if (e <= EDYN_EPSILON) {
            // `s2` degenerates into a point.
            t = 0;
            s = std::clamp(-c / a, scalar(0), scalar(1));
        } else {
            // The general non-degenerate case starts here.
            const auto b = dot(d1, d2);
            const auto denom = a * e - b * b;
            
            // If segments aren't parallel, compute closest point on `l1` to `l2`
            // and clamp to `s1`. Else pick arbitrary `s` (here 0). `l1` and `l2`
            // are the infinite lines passing through `s1` and `s2` respectively.
            if (denom > EDYN_EPSILON) {
                s = std::clamp((b * f - c * e) / denom, scalar(0), scalar(1));
            } else {
                s = 0;
            }

            // Find point on `l2` closeset to `s1(s)` using
            // `t = dot((p1 + d1 * s) - p2, d2) / dot(d2, d2) = (b * s + f) / e`.
            const auto tnom = b * s + f;

            // If `t` in [0, 1] we're done. Else clamp `t`, recompute `s` for
            // the new value of `t` using 
            // `s = dot((p2 + d2 * t) - p1, d1) / dot(d1, d1) = (t * b - c) / a`
            // and clamp `s` to [0, 1].
            if (t < 0) {
                t = 0;
                s = std::clamp(-c / a, scalar(0), scalar(1));
            } else if (tnom > e) {
                t = 1;
                s = std::clamp((b - c) / a, scalar(0), scalar(1));
            } else {
                t = tnom / e;
            }
        }
    }

    c1 = p1 + d1 * s;
    c2 = p2 + d2 * t;
    return length2(c1 - c2);
}

}