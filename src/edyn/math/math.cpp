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
    t = clamp_unit(a / b);
    q = q0 + v * t;
    return length2(p - q);
}

// Reference: Real-Time Collision Detection - Christer Ericson, section 5.1.9.
scalar closest_point_segment_segment(const vector3 &p1, const vector3 &q1, 
                                     const vector3 &p2, const vector3 &q2, 
                                     scalar &s, scalar &t, 
                                     vector3 &c1, vector3 &c2,
                                     size_t *num_points,
                                     scalar *sp, scalar *tp, 
                                     vector3 *c1p, vector3 *c2p) {
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
        t = clamp_unit(t);
    } else {
        auto c = dot(d1, r);

        if (e <= EDYN_EPSILON) {
            // `s2` degenerates into a point.
            t = 0;
            s = clamp_unit(-c / a);
        } else {
            // The general non-degenerate case starts here.
            const auto b = dot(d1, d2);
            const auto denom = a * e - b * b;
            
            // If segments aren't parallel, compute closest point on `l1` to `l2`
            // and clamp to `s1`. Else pick arbitrary `s` (here 0). `l1` and `l2`
            // are the infinite lines passing through `s1` and `s2` respectively.
            if (denom > EDYN_EPSILON) {
                s = clamp_unit((b * f - c * e) / denom);
                if (num_points != nullptr) {
                    *num_points = 1;
                }
            } else if (num_points != nullptr) {
                auto r1 = p1 - q2;
                auto c1 = dot(d1, r1);
                auto a_inv = 1 / a;
                
                s   = clamp_unit(std::min(-c * a_inv, -c1 * a_inv));
                *sp = clamp_unit(std::max(-c * a_inv, -c1 * a_inv));

                auto r2 = q1 - p2;
                auto f2 = dot(d2, r2);
                auto e_inv = 1 / e;

                t   = clamp_unit(std::min(f * e_inv, f2 * e_inv));
                *tp = clamp_unit(std::max(f * e_inv, f2 * e_inv));
                
                if (std::abs(s - *sp) > EDYN_EPSILON) {
                    *num_points = 2;
                    *c1p = p1 + d1 * *sp;
                    *c2p = p2 + d2 * *tp;
                } else {
                    *num_points = 1;
                }
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
            if (tnom < 0) {
                t = 0;
                s = clamp_unit(-c / a);
            } else if (tnom > e) {
                t = 1;
                s = clamp_unit((b - c) / a);
            } else {
                t = tnom / e;
            }
        }
    }

    c1 = p1 + d1 * s;
    c2 = p2 + d2 * t;
    return length2(c1 - c2);
}

scalar closest_point_disc_segment(const vector3 &cpos, const quaternion &corn,  scalar radius,
                                  const vector3 &p0, const vector3 &p1, size_t &num_points, 
                                  scalar &s0, vector3 &cc0, vector3 &cs0,
                                  scalar &s1, vector3 &cc1, vector3 &cs1, 
                                  vector3 &normal) {
    // Segment vertices in local disc space.
    auto corn_conj = conjugate(corn);
    auto q0 = rotate(corn_conj, p0 - cpos);
    auto q1 = rotate(corn_conj, p1 - cpos);

    constexpr scalar dl = 0.01;
    auto len = length(p0 - p1);
    auto ds = dl / len;
    scalar s = 0;
    auto dq = q1 - q0;
    auto dist2 = EDYN_SCALAR_MAX;

    for (;;) {
        auto q = q0 + dq * s;
        auto r = q;
        r.x = 0;
        r = normalize(r) * radius;
        auto d = r - q;
        auto l2 = length2(d);

        if (l2 < dist2) {
            dist2 = l2;
            s0 = s;
            cc0 = cpos + rotate(corn, r);
            cs0 = cpos + rotate(corn, q);
            normal = l2 > EDYN_EPSILON ? d / std::sqrt(l2) : vector3_x;
            normal = rotate(corn, normal); 
        }

        if (s >= 1) { break; }
        s = std::min(s + ds, scalar(1));
    }

    num_points = 1;

    return dist2;
}

scalar closest_point_disc_disc(const vector3 &posA, const quaternion &ornA, scalar radiusA,
                               const vector3 &posB, const quaternion &ornB, scalar radiusB,
                               size_t &num_points, closest_points_array &closest, 
                               vector3 &normal) {
    // Transform disc B onto disc A's space.
    auto ornA_conj = conjugate(ornA);
    auto ornBA = ornA_conj * ornB;
    auto posBA = rotate(ornA_conj, posB - posA);

    // Build ortho basis on B (in A's space).
    auto u = rotate(ornBA, vector3_y) * radiusB;
    auto v = rotate(ornBA, vector3_z) * radiusB;

    scalar s = 0;
    scalar ds = pi / 18;
    auto dist2 = EDYN_SCALAR_MAX;

    for (;;) {
        auto q = posBA + u * std::cos(s) + v * std::sin(s);
        auto r = q;
        r.x = 0;
        r = normalize(r) * radiusA;
        auto d = r - q;
        auto l2 = length2(d);

        if (l2 < dist2) {
            dist2 = l2;
            closest[0].first = posA + rotate(ornA, r);
            closest[0].second = posA + rotate(ornA, q);
            normal = l2 > EDYN_EPSILON ? d / std::sqrt(l2) : vector3_x;
            normal = rotate(ornA, normal);
        }

        if (s >= 2 * pi) { break; }
        s = std::min(s + ds, 2 * pi);
    }

    num_points = 1;

    return dist2;
}

}