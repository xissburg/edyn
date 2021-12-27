#include "edyn/math/geom.hpp"
#include "edyn/math/math.hpp"
#include "edyn/math/vector2_3_util.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/math/transform.hpp"
#include "edyn/util/triangle_util.hpp"
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
    return length_sqr(p - q);
}

scalar distance_sqr_line(const vector3 &q0, const vector3 &dir,
                         const vector3 &p) {
    auto w = p - q0;
    auto a = dot(w, dir);
    auto b = dot(dir, dir);
    EDYN_ASSERT(b > EDYN_EPSILON);
    auto t = a / b;
    auto q = q0 + dir * t;
    return length_sqr(p - q);
}

scalar closest_point_line(const vector3 &q0, const vector3 &dir,
                          const vector3 &p, scalar &t, vector3 &r) {
    auto w = p - q0; // Vector from initial point of line to point `p`.
    auto a = dot(w, dir);
    auto b = dot(dir, dir);
    EDYN_ASSERT(b > EDYN_EPSILON);
    t = a / b;
    r = q0 + dir * t;
    return length_sqr(p - r);
}

bool closest_point_line_line(const vector3 &p1, const vector3 &q1,
                             const vector3 &p2, const vector3 &q2,
                             scalar &s, scalar &t) {
    // Reference: Real-Time Collision Detection - Christer Ericson,
    // Section 5.1.8 - Closest Points of Two Lines
    auto d1 = q1 - p1;
    auto d2 = q2 - p2;
    auto r = p1 - p2;
    auto a = dot(d1, d1);
    auto b = dot(d1, d2);
    auto c = dot(d1, r);
    auto e = dot(d2, d2);
    auto f = dot(d2, r);
    auto d = a * e - b * b;

    if (!(d > EDYN_EPSILON)) {
        return false;
    }

    auto d_inv = scalar(1) / d;
    s = (b * f - c * e) * d_inv;
    t = (a * f - b * c) * d_inv;

    return true;
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
        return length_sqr(c1 - c2);
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
                auto f1 = dot(d1, r1);
                auto a_inv = 1 / a;

                s   = clamp_unit(std::min(-c * a_inv, -f1 * a_inv));
                *sp = clamp_unit(std::max(-c * a_inv, -f1 * a_inv));

                auto r2 = p2 - q1;
                auto f2 = dot(d2, r2);
                auto e_inv = 1 / e;

                t   = clamp_unit(std::min(-f * e_inv, -f2 * e_inv));
                *tp = clamp_unit(std::max(-f * e_inv, -f2 * e_inv));

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
    return length_sqr(c1 - c2);
}

scalar closest_point_disc(const vector3 &dpos, const quaternion &dorn,
                          scalar radius, const vector3 &p, vector3 &q) {
    // Project point onto disc's plane.
    const auto normal = rotate(dorn, vector3_x);
    const auto ln = dot(p - dpos, normal);
    const auto p_proj = p - normal * ln;
    const auto d = p_proj - dpos;
    const auto l2 = length_sqr(d);

    // Projection is inside disc.
    if (l2 < radius * radius) {
        q = p_proj;
        return ln * ln;
    }

    const auto l = std::sqrt(l2);
    const auto dn = d / l;
    q = dpos + dn * radius;
    return length_sqr(p - q);
}

size_t intersect_line_circle(const vector2 &p0, const vector2 &p1,
                             scalar radius, scalar &s0, scalar &s1) {
    auto d = p1 - p0;
    auto dl2 = length_sqr(d);
    auto dp = dot(d, p0);
    auto delta = dp * dp - dl2 * (dot(p0, p0) - radius * radius);

    if (delta < 0) {
        return 0;
    }

    if (delta > EDYN_EPSILON) {
        auto delta_sqrt = std::sqrt(delta);
        auto dl2_inv = 1 / dl2;
        s0 = -(dp + delta_sqrt) * dl2_inv;
        s1 = -(dp - delta_sqrt) * dl2_inv;
        return 2;
    }

    s0 = -dp * dl2;
    return 1;
}

scalar closest_point_circle_line(
    const vector3 &cpos, const quaternion &corn, scalar radius,
    const vector3 &p0, const vector3 &p1, size_t &num_points,
    scalar &s0, vector3 &rc0, vector3 &rl0,
    scalar &s1, vector3 &rc1, vector3 &rl1,
    vector3 &normal, scalar /*threshold*/) {

    // Line points in local disc space. The face of the disc points towards
    // the positive x-axis.
    auto q0 = to_object_space(p0, cpos, corn);
    auto q1 = to_object_space(p1, cpos, corn);
    auto qv = q1 - q0;
    auto qv_len_sqr = length_sqr(qv);

    // If the projection of a segment of the line of length `diameter` on the x axis
    // is smaller than threshold, the line is considered to be parallel to the circle.
    if (std::abs(qv.x) < EDYN_EPSILON) { // (std::abs(qv.x / qv_len) * diameter < threshold) {
        // Calculate line-circle intersection in the yz plane.
        // For the normal vector, calculate something orthogonal to the line.
        auto tangent = cross(qv, vector3_x); // tangent lies on the circle plane.
        normal = cross(qv, tangent);
        normal = rotate(corn, normal);
        normal = normalize(normal);
        EDYN_ASSERT(length_sqr(normal) > EDYN_EPSILON);

        num_points = intersect_line_circle(to_vector2_yz(q0), to_vector2_yz(q1), radius, s0, s1);

        if (num_points > 0) {
            auto rl0_local = q0 + qv * s0;
            auto rc0_local = vector3{0, rl0_local.y, rl0_local.z};
            // Transform to world space.
            rl0 = cpos + rotate(corn, rl0_local);
            rc0 = cpos + rotate(corn, rc0_local);

            // The distance is simply the x coord of the resulting point in the line
            // in circle space.
            auto dist2 = rl0_local.x * rl0_local.x;

            if (num_points > 1) {
                auto rl1_local = q0 + qv * s1;
                auto rc1_local = vector3{0, rl1_local.y, rl1_local.z};
                rl1 = cpos + rotate(corn, rl1_local);
                rc1 = cpos + rotate(corn, rc1_local);

                dist2 = std::min(dist2, rl1_local.x * rl1_local.x);
            }

            return dist2;
        } else {
            // If the projection of line in the yz plane does not intersect disc
            // (despite being parallel), the closest point in the line is the point
            // closest to the circle center and the the closest point on the circle
            // is the closest point on the line projected on the circle plane,
            // normalized and multiplied by radius.
            // Calculations done in world-space this time.
            closest_point_line(p0, p1 - p0, cpos, s0, rl0);
            auto proj = project_plane(rl0, cpos, normal);
            auto dir = normalize(proj - cpos);
            rc0 = cpos + dir * radius;
            auto d = rl0 - rc0;
            auto dl2 = length_sqr(d);

            if (dl2 > EDYN_EPSILON) {
                normal = d / std::sqrt(dl2);
            } else {
                normal = dir;
            }

            EDYN_ASSERT(length_sqr(normal) > EDYN_EPSILON);
            num_points = 1;
            return dl2;
        }
    }

    // The root finder below would fail if the line is orthogonal to the plane of the
    // circle and is also centered at the circle.
    if (length_sqr(to_vector2_yz(q0)) <= EDYN_EPSILON &&
        length_sqr(to_vector2_yz(q1)) <= EDYN_EPSILON ) {
        num_points = 1;
        normal = quaternion_z(corn);
        s0 = -q0.x / qv.x;
        rc0 = cpos + normal * radius;
        rl0 = lerp(p0, p1, s0);
        return radius * radius;
    }

    // Let `q(θ) = [0, sin(θ) * r, cos(θ) * r]` be the parametrized circle in the yz
    // plane, then the function `c(θ) = q0 + ((q(θ) - q0) · qv) / (qv · qv) * qv`
    // gives the point in the line that's closest to `q(θ)`. The function
    // `d(θ) = q(θ) - c(θ)` gives us the vector connecting the closest points.
    // The function `f(θ) = 0.5 * d(θ) · d(θ)` gives us half the squared length of
    // `d(θ)`. Minimizing `f(θ)` will result in the angle `θ_m` where `d(θ)` achieves
    // its smallest magnitude, thus `q(θ_m)` is the point in the circle closest to
    // the line. The Newton-Raphson method is used for minimization.

    // Intersect line with yz plane and use the angle of this point as the
    // initial value.
    const auto q_yz_plane = q0 - (q0.x / qv.x) * qv;
    const auto initial_theta = std::atan2(q_yz_plane.y, q_yz_plane.z);
    const auto qv_len_sqr_inv = scalar(1) / qv_len_sqr;

    // Newton-Raphson iterations.
    auto theta = initial_theta;
    size_t max_iterations = 20;

    for (size_t i = 0; i < max_iterations; ++i) {
        auto sin_theta = std::sin(theta);
        auto cos_theta = std::cos(theta);

        // Function q(θ) = [0, sin(θ) * r, cos(θ) * r] and its first and
        // second derivatives.
        auto q_theta    = vector3{0,  sin_theta * radius,  cos_theta * radius};
        auto d_q_theta  = vector3{0,  cos_theta * radius, -sin_theta * radius};
        auto dd_q_theta = vector3{0, -sin_theta * radius, -cos_theta * radius};

        // Function c(θ) gives the point in the line closest to q(θ).
        // First and second derivates follow.
        auto c_theta = q0 + dot(q_theta - q0, qv) * qv_len_sqr_inv * qv;
        auto d_c_theta = dot(d_q_theta, qv) * qv_len_sqr_inv * qv;
        auto dd_c_theta = dot(dd_q_theta, qv) * qv_len_sqr_inv * qv;

        // Function d(θ) is the vector connecting the closest points.
        // First and second derivates follow.
        auto d_theta = q_theta - c_theta;
        auto d_d_theta = d_q_theta - d_c_theta;
        auto dd_d_theta = dd_q_theta - dd_c_theta;

        // Function f(θ) = d · d / 2 gives us a scalar proportional to the
        // length of d(θ).
        // First derivative f' = d'· d
        // Second derivative f" = d'· d + d"· d
        // auto f_theta = scalar(0.5) * dot(d_theta, d_theta);
        auto d_f_theta = dot(d_theta, d_d_theta);
        auto dd_f_theta = dot(d_d_theta, d_d_theta) + dot(dd_d_theta, d_theta);

        EDYN_ASSERT(dd_f_theta > 0 || dd_f_theta < 0);

        auto delta = d_f_theta / dd_f_theta;
        theta -= delta;

        // Stop when the delta is smaller than a degree.
        if (std::abs(delta) < pi * scalar(1) / scalar(180)) {
            break;
        }
    }

    auto closest_sin_theta = std::sin(theta);
    auto closest_cos_theta = std::cos(theta);
    auto rc0_local = vector3{0, closest_sin_theta * radius, closest_cos_theta * radius};
    vector3 rl0_local;
    auto dist_sqr = closest_point_line(q0, qv, rc0_local, s0, rl0_local);

    rc0 = cpos + rotate(corn, rc0_local);
    rl0 = cpos + rotate(corn, rl0_local);

    // Get tangent at θ and use cross product with line to calculate normal.
    auto tangent = vector3{0, closest_cos_theta, -closest_sin_theta};
    normal = cross(tangent, qv);

    auto normal_len_sqr = length_sqr(normal);

    if (normal_len_sqr > EDYN_EPSILON) {
        normal /= std::sqrt(normal_len_sqr);
        normal = rotate(corn, normal);
    } else if (dist_sqr > EDYN_EPSILON) {
        // If line is parallel to tangent and closest points do not coincide.
        normal = (rl0 - rc0) / std::sqrt(dist_sqr);
    } else {
        // If points coincide, take a vector at an angle θ.
        normal = vector3{0, closest_sin_theta, closest_cos_theta};
        normal = rotate(corn, normal);
    }

    EDYN_ASSERT(length_sqr(normal) > EDYN_EPSILON);

    num_points = 1;

    return dist_sqr;
}

size_t intersect_circle_circle(const vector2 &posA, scalar radiusA,
                               const vector2 &posB, scalar radiusB,
                               vector2 &res0, vector2 &res1) {
    // Reference: Intersection of Linear and Circular Components in 2D - David Eberly
    // https://www.geometrictools.com/Documentation/IntersectionLine2Circle2.pdf
    auto u = posB - posA;
    auto lu2 = length_sqr(u);
    auto rsum = radiusA + radiusB;
    auto rsub = radiusA - radiusB;

    if (lu2 < EDYN_EPSILON && rsub < EDYN_EPSILON) {
        // Concentric circles of same radius.
        res0 = posA + vector2_x * radiusA;
        res1 = posB - vector2_x * radiusB;
        return 2;
    }

    if (lu2 < rsub * rsub || lu2 > rsum * rsum) {
        return 0;
    }

    auto lu2_inv = scalar(1) / lu2;
    auto s = ((radiusA * radiusA - radiusB * radiusB) * lu2_inv + scalar(1)) * scalar(0.5);
    auto t = std::sqrt(std::max(scalar(0), radiusA * radiusA * lu2_inv - s * s));

    auto v = orthogonal(u);
    auto su = s * u;
    auto tv = t * v;

    res0 = posA + su + tv;
    res1 = posA + su - tv;

    return t > EDYN_EPSILON ? 2 : 1;
}

scalar closest_point_circle_circle(
    const vector3 &posA, const quaternion &ornA, scalar radiusA,
    const vector3 &posB, const quaternion &ornB, scalar radiusB,
    size_t &num_points, vector3 &rA0, vector3 &rB0, vector3 &rA1, vector3 &rB1,
    vector3 &normal) {

    auto normalA = rotate(ornA, vector3_x);
    auto normalB = rotate(ornB, vector3_x);
    auto posB_in_A = to_object_space(posB, posA, ornA);

    // Check if parallel.
    if (!(length_sqr(cross(normalA, normalB)) > EDYN_EPSILON)) {
        normal = normalB;

        vector2 c0, c1;
        // A is in the origin.
        auto np = intersect_circle_circle(vector2_zero, radiusA,
                                          to_vector2_zy(posB_in_A), radiusB,
                                          c0, c1);
        if (np > 0) {
            num_points = np;
            rA0 = posA + rotate(ornA, vector3 {0, c0.y, c0.x});
            rB0 = posA + rotate(ornA, vector3 {posB_in_A.x , c0.y, c0.x});

            if (np > 1) {
                rA1 = posA + rotate(ornA, vector3 {0, c1.y, c1.x});
                rB1 = posA + rotate(ornA, vector3 {posB_in_A.x , c1.y, c1.x});
            }

            return posB_in_A.x * posB_in_A.x;
        } else {
            // One circle could be contained within the other.
            // If the circles do not intersect and any point of one of them is
            // contained in the other, then the entire circle is contained in
            // the other.

            num_points = 1;
            auto dir = vector2{posB_in_A.z, posB_in_A.y};
            auto dir_len_sqr = length_sqr(dir);

            if (dir_len_sqr > EDYN_EPSILON) {
                dir /= std::sqrt(dir_len_sqr);
                auto pointB_in_A = posB_in_A + vector3_y * radiusB;
                auto pointA = vector3_y * radiusA;

                if (length_sqr(vector2{pointB_in_A.z, pointB_in_A.y}) < radiusA * radiusA) {
                    // B contained in A.
                    rA0 = posA + rotate(ornA, vector3{0, dir.y * radiusA, dir.x * radiusA});
                    rB0 = posA + rotate(ornA, posB_in_A + vector3{0, dir.y * radiusB, dir.x * radiusB});
                    return distance_sqr(rA0, rB0);
                } else if (length_sqr(vector2{pointA.z, pointA.y}) < radiusB * radiusB) {
                    // A contained in B.
                    rA0 = posA + rotate(ornA, vector3{0, -dir.y * radiusA, -dir.x * radiusA});
                    rB0 = posA + rotate(ornA, posB_in_A + vector3{0, -dir.y * radiusB, -dir.x * radiusB});
                    return distance_sqr(rA0, rB0);
                } else {
                    // Circles don't intersect nor are contained in one another.
                    rA0 = posA + rotate(ornA, vector3{0, dir.y * radiusA, dir.x * radiusA});
                    rB0 = posA + rotate(ornA, posB_in_A + vector3{0, -dir.y * radiusB, -dir.x * radiusB});
                    return distance_sqr(rA0, rB0);
                }
            } else {
                // Concentric.
                rA0 = posA + rotate(ornA, vector3_y * radiusA);
                rB0 = posB + rotate(ornB, vector3_y * radiusB);
                return distance_sqr(rA0, rB0);
            }
        }
    }

    // Let `q(θ) = [0, sin(θ) * r, cos(θ) * r]` be the parametrized circle of
    // radius `r` in the yz plane, and `p(φ) = u * cos(φ) * s + v * sin(φ) * s`
    // be the other parametrized circle of radius `s` where `u` and `v` are
    // unit orthogonal vectors. The `θ` which gives us the point in `q(θ)`
    // closest to any point `a` is `atan(a_y / a_z)`, thus `θ` can be written
    // as a function of `φ`: `θ(φ) = atan(p_y(φ) / p_z(φ))`. Then, a function
    // which gives us the vector connecting the closest points can be written
    // `d(φ) = p(φ) - q(θ(φ))` and a function that gives us a quantity proportinal
    // to the distance between these points can be written `f(φ) = d(φ) · d(φ) / 2`.
    // Minimizing `f(φ)` will result in the value of `φ` that gives us the closest
    // points between the two circles.

    // Build ortho basis on B (in A's space).
    auto ornB_in_A = conjugate(ornA) * ornB;
    auto u = quaternion_z(ornB_in_A);
    auto v = quaternion_y(ornB_in_A);

    // Use angle of support point of B closest on A's plane as initial angle.
    vector3 sup_pos = support_point_circle(posB_in_A, ornB_in_A, radiusB, vector3_x);
    vector3 sup_neg = support_point_circle(posB_in_A, ornB_in_A, radiusB, -vector3_x);
    vector3 sup = std::abs(sup_pos.x) < std::abs(sup_neg.x) ? sup_pos : sup_neg;
    auto sup_in_B = to_object_space(sup, posB_in_A, ornB_in_A);
    auto initial_phi = std::atan2(sup_in_B.y, sup_in_B.z);

    // Newton-Raphson iterations.
    auto phi = initial_phi;
    size_t max_iterations = 20;

    for (size_t i = 0; i < max_iterations; ++i) {
        auto cos_phi = std::cos(phi);
        auto sin_phi = std::sin(phi);

        auto p_phi = posB_in_A + (u * cos_phi + v * sin_phi) * radiusB;
        auto d_p_phi = (u * -sin_phi + v * cos_phi) * radiusB;
        auto dd_p_phi = (u * -cos_phi + v * -sin_phi) * radiusB;

        auto theta = std::atan2(p_phi.y, p_phi.z);
        auto cos_theta = std::cos(theta);
        auto sin_theta = std::sin(theta);

        auto q_theta = vector3{0, sin_theta * radiusA, cos_theta * radiusA};
        auto d_q_theta = vector3{0, cos_theta * radiusA, -sin_theta * radiusA};
        auto dd_q_theta = vector3{0, -sin_theta * radiusA, -cos_theta * radiusA};

        // Function d(φ) is the vector connecting the closest points.
        // First and second derivates follow.
        auto d_phi = p_phi - q_theta;
        auto d_d_phi = d_p_phi - d_q_theta;
        auto dd_d_phi = dd_p_phi - dd_q_theta;

        // Function f(φ) = d · d / 2 gives us a scalar proportional to the
        // length of d(φ).
        // First derivative f' = d'· d
        // Second derivative f" = d'· d + d"· d
        // auto f_theta = scalar(0.5) * dot(d_phi, d_phi);
        auto d_f_phi = dot(d_phi, d_d_phi);
        auto dd_f_phi = dot(d_d_phi, d_d_phi) + dot(dd_d_phi, d_phi);

        auto delta = d_f_phi / dd_f_phi;
        phi -= delta;

        // Stop when the delta is smaller than a degree.
        if (std::abs(delta) < pi * scalar(1) / scalar(180)) {
            break;
        }
    }

    auto cos_phi = std::cos(phi);
    auto sin_phi = std::sin(phi);
    rB0 = posB_in_A + (u * cos_phi + v * sin_phi) * radiusB;

    auto theta = std::atan2(rB0.y, rB0.z);
    auto cos_theta = std::cos(theta);
    auto sin_theta = std::sin(theta);
    rA0 = vector3{0, sin_theta * radiusA, cos_theta * radiusA};

    rA0 = posA + rotate(ornA, rA0);
    rB0 = posA + rotate(ornA, rB0);
    auto dir = rA0 - rB0;
    auto dist_sqr = length_sqr(dir);

    // Get tangents and use cross product to calculate normal.
    auto tangentA = vector3{0, cos_theta, -sin_theta};
    auto tangentB = u * -sin_phi + v * cos_phi;
    normal = cross(tangentA, tangentB);

    auto normal_len_sqr = length_sqr(normal);

    if (normal_len_sqr > EDYN_EPSILON) {
        normal /= std::sqrt(normal_len_sqr);
        normal = rotate(ornA, normal);
    } else if (dist_sqr > EDYN_EPSILON) {
        // If line is parallel to tangent and closest points do not coincide.
        normal = dir / std::sqrt(dist_sqr);
    } else {
        // If points coincide, take a vector at an angle θ.
        normal = vector3{0, sin_theta, cos_theta};
        normal = rotate(ornA, normal);
    }

    EDYN_ASSERT(length_sqr(normal) > EDYN_EPSILON);

    num_points = 1;

    return dist_sqr;
}

void plane_space(const vector3 &n, vector3 &p, vector3 &q) {
    if (std::abs(n.z) > half_sqrt2) {
        // Choose p in yz plane.
        auto a = n.y * n.y + n.z * n.z;
        auto k = scalar(1) / std::sqrt(a);
        p.x = 0;
        p.y = -n.z * k;
        p.z = n.y * k;
        // q = n X p
        q.x = a * k;
        q.y = -n.x * p.z;
        q.z = n.x * p.y;
    } else {
        // Choose p in xy plane.
        auto a = n.x * n.x + n.y * n.y;
        auto k = scalar(1) / std::sqrt(a);
        p.x = -n.y * k;
        p.y = n.x * k;
        p.z = 0;
        // q = n X p
        q.x = -n.z * p.y;
        q.y = n.z * p.x;
        q.z = a * k;
    }
}

matrix3x3 make_tangent_basis(const vector3 &n) {
    vector3 t, u;
    plane_space(n, t, u);
    return matrix3x3_columns(t, n, u);
}

bool intersect_aabb(const vector3 &min0, const vector3 &max0,
                    const vector3 &min1, const vector3 &max1) {
    return (min0.x <= max1.x) &&
		   (max0.x >= min1.x) &&
		   (min0.y <= max1.y) &&
		   (max0.y >= min1.y) &&
		   (min0.z <= max1.z) &&
		   (max0.z >= min1.z);
}

vector3 support_point_circle(const vector3 &pos, const quaternion &orn,
                             scalar radius, const vector3 &dir) {
    auto local_dir = rotate(conjugate(orn), dir);
    // Squared length in yz plane.
    auto len_yz_sqr = local_dir.y * local_dir.y + local_dir.z * local_dir.z;
    vector3 sup;

    if (len_yz_sqr > EDYN_EPSILON) {
        auto d = radius / std::sqrt(len_yz_sqr);
        sup = {0, local_dir.y * d, local_dir.z * d};
    } else {
        sup = {0, radius, 0};
    }

    return pos + rotate(orn, sup);
}

scalar signed_triangle_area(const vector2 &a, const vector2 &b, const vector2 &c) {
    return (a.x - c.x) * (b.y - c.y) - (a.y - c.y) * (b.x - c.x);
}

size_t intersect_segments(const vector2 &p0, const vector2 &p1,
                          const vector2 &q0, const vector2 &q1,
                          scalar &s0, scalar &t0,
                          scalar &s1, scalar &t1) {
    auto dp = p1 - p0;
    auto dq = q1 - q0;
    auto e = q0 - p0;
    auto denom = perp_product(dp, dq);

    if (std::abs(denom) > EDYN_EPSILON) {
        auto denom_inv = scalar(1) / denom;
        s0 = perp_product(e, dq) * denom_inv;
        t0 = perp_product(e, dp) * denom_inv;
        return s0 < 0 || s0 > 1 || t0 < 0 || t0 > 1 ? 0 : 1;
    }

    if (std::abs(perp_product(e, dp)) < EDYN_EPSILON) {
        // Segments are parallel and lie on the same line.
        // Calculate intersection interval.
        auto denom_p = scalar(1) / dot(dp, dp);
        auto denom_q = scalar(1) / dot(dq, dq);

        s0 = dot(q0 - p0, dp) * denom_p;
        s1 = dot(q1 - p0, dp) * denom_p;

        if ((s0 < 0 && s1 < 0) || (s0 > 1 && s1 > 1)) {
            // Segments do not overlap.
            return 0;
        }

        s0 = clamp_unit(s0);
        s1 = clamp_unit(s1);

        t0 = clamp_unit(dot(p0 - q0, dq) * denom_q);
        t1 = clamp_unit(dot(p1 - q0, dq) * denom_q);

        return std::abs(s1 - s0) < EDYN_EPSILON ? 1 : 2;
    }

    return 0;
}

scalar area_4_points(const vector3 &p0, const vector3 &p1, const vector3 &p2, const vector3 &p3) {
	vector3 a[3], b[3];
	a[0] = p0 - p1;
	a[1] = p0 - p2;
	a[2] = p0 - p3;
	b[0] = p2 - p3;
	b[1] = p1 - p3;
	b[2] = p1 - p2;

	vector3 tmp0 = cross(a[0], b[0]);
	vector3 tmp1 = cross(a[1], b[1]);
	vector3 tmp2 = cross(a[2], b[2]);

	return std::max(std::max(length_sqr(tmp0), length_sqr(tmp1)), length_sqr(tmp2));
}

insertion_point_result insertion_point_index(const vector3 *points,
                                             const scalar *depths,
                                             size_t count,
                                             size_t &num_points,
                                             const vector3 &new_point,
                                             scalar new_point_depth) {
    EDYN_ASSERT(num_points <= count);
    const auto max_dist_similar_sqr = contact_merging_threshold * contact_merging_threshold;

    if (num_points < 2) {
        return {point_insertion_type::append, num_points++};
    }

    if (num_points == 2) {
        // Check if collinear.
        if (length_sqr(cross(new_point - points[0], new_point - points[1])) > EDYN_EPSILON) {
            return {point_insertion_type::append, num_points++};
        } else {
            // Select a point to replace. Maximize segment length.
            auto dist_sqr0 = distance_sqr(new_point, points[0]);
            auto dist_sqr1 = distance_sqr(new_point, points[1]);
            auto curr_dist_sqr = distance_sqr(points[0], points[1]);

            if (dist_sqr0 > curr_dist_sqr && dist_sqr0 > dist_sqr1) {
                auto type = dist_sqr1 < max_dist_similar_sqr ?
                    point_insertion_type::similar : point_insertion_type::replace;
                return {type, 1};
            } else if (dist_sqr1 > curr_dist_sqr && dist_sqr1 > dist_sqr0) {
                auto type = dist_sqr0 < max_dist_similar_sqr ?
                    point_insertion_type::similar : point_insertion_type::replace;
                return {type, 0};
            } else {
                // Ignore new point because the current contact set is better as it is.
                return {point_insertion_type::none, count};
            }
        }
    }

    if (num_points == 3) {
        auto vertices = std::array<vector3, 3>{points[0], points[1], points[2]};
        auto normal = cross(points[0] - points[1], points[1] - points[2]);

        if (try_normalize(normal)) {
            if (point_in_triangle(vertices, normal, new_point)) {
                // Ignore new point because it's inside the existing contact region.
                return {point_insertion_type::none, count};
            } else {
                return {point_insertion_type::append, num_points++};
            }
        } else {
            // Points are collinear. Replace point in the middle.
            auto d0 = dot(points[1] - points[0], points[2] - points[0]);

            if (d0 > 0 && d0 < 1) {
                // Point 1 is between 0 and 2.
                return {point_insertion_type::replace, 1};
            }

            auto d1 = dot(points[0] - points[1], points[2] - points[1]);

            if (d1 > 0 && d1 < 1) {
                // Point 0 is between 1 and 2.
                return {point_insertion_type::replace, 0};
            }

            auto d2 = dot(points[2] - points[0], points[1] - points[0]);

            if (d2 > 0 && d2 < 1) {
                // Point 2 is between 0 and 1.
                return {point_insertion_type::replace, 2};
            }

            // Points coincide. Find them and replace one.
            std::array<scalar, 3> dist_sqr;
            dist_sqr[0] = distance_sqr(points[0], points[1]);
            dist_sqr[1] = distance_sqr(points[1], points[2]);
            dist_sqr[2] = distance_sqr(points[2], points[0]);

            size_t min_dist_idx = SIZE_MAX;
            auto min_dist_sqr = EDYN_SCALAR_MAX;

            for (size_t i = 0; i < dist_sqr.size(); ++i) {
                if (dist_sqr[i] < min_dist_sqr) {
                    min_dist_sqr = dist_sqr[i];
                    min_dist_idx = i;
                }
            }

            return {point_insertion_type::replace, min_dist_idx};
        }
    }

    // The approximate area when the i-th point is removed.
    auto areas = make_array<4>(scalar(0));
    areas[0] = area_4_points(new_point, points[1], points[2], points[3]);
    areas[1] = area_4_points(new_point, points[0], points[2], points[3]);
    areas[2] = area_4_points(new_point, points[0], points[1], points[3]);
    areas[3] = area_4_points(new_point, points[0], points[1], points[2]);

    auto current_area = area_4_points(points[0], points[1], points[2], points[3]);
    auto max_area = current_area;
    auto max_area_idx = SIZE_MAX;

    for (size_t i = 0; i < areas.size(); ++i) {
        if (areas[i] > max_area) {
            max_area = areas[i];
            max_area_idx = i;
        }
    }

    if (max_area_idx < max_contacts) {
        auto type = distance_sqr(points[max_area_idx], new_point) < max_dist_similar_sqr ?
                    point_insertion_type::similar : point_insertion_type::replace;
        return {type, max_area_idx};
    }

    // Ignore new point because the current contact set is better as it is.
    return {point_insertion_type::none, count};
}

vector3 closest_point_box_outside(const vector3 &half_extent, const vector3 &p) {
    auto closest = p;
    closest.x = std::min(half_extent.x, closest.x);
    closest.x = std::max(-half_extent.x, closest.x);
    closest.y = std::min(half_extent.y, closest.y);
    closest.y = std::max(-half_extent.y, closest.y);
    closest.z = std::min(half_extent.z, closest.z);
    closest.z = std::max(-half_extent.z, closest.z);
    return closest;
}

scalar closest_point_box_inside(const vector3 &half_extent, const vector3 &p, vector3 &closest, vector3 &normal) {
    EDYN_ASSERT(p >= -half_extent && p <= half_extent);

    auto dist = half_extent.x - p.x;
    auto min_dist = dist;
    closest = vector3{half_extent.x, p.y, p.z};
    normal = vector3{1, 0, 0};

    dist = half_extent.x + p.x;
    if (dist < min_dist) {
        min_dist = dist;
        closest = vector3{-half_extent.x, p.y, p.z};
        normal = vector3{-1, 0, 0};
    }

    dist = half_extent.y - p.y;
    if (dist < min_dist) {
        min_dist = dist;
        closest = vector3{p.x, half_extent.y, p.z};
        normal = vector3{0, 1, 0};
    }

    dist = half_extent.y + p.y;
    if (dist < min_dist) {
        min_dist = dist;
        closest = vector3{p.x, -half_extent.y, p.z};
        normal = vector3{0, -1, 0};
    }

    dist = half_extent.z - p.z;
    if (dist < min_dist) {
        min_dist = dist;
        closest = vector3{p.x, p.y, half_extent.z};
        normal = vector3{0, 0, 1};
    }

    dist = half_extent.z + p.z;
    if (dist < min_dist) {
        min_dist = dist;
        closest = vector3{p.x, p.y, -half_extent.z};
        normal = vector3{0, 0, -1};
    }

    return dist;
}

size_t intersect_line_aabb(const vector2 &p0, const vector2 &p1,
                           const vector2 &aabb_min, const vector2 &aabb_max,
                           scalar &s0, scalar &s1) {
    size_t num_points = 0;
    auto d = p1 - p0;
    auto e = aabb_min - p0;
    auto f = aabb_max - p0;

    if (std::abs(d.x) < EDYN_EPSILON) {
        // Line is vertical.
        if (e.x <= 0 && f.x >= 0) {
            s0 = e.y / d.y;
            s1 = f.y / d.y;
            num_points = 2;
        }

        return num_points;
    }

    if (std::abs(d.y) < EDYN_EPSILON) {
        // Line is horizontal.
        if (e.y <= 0 && f.y >= 0) {
            s0 = e.x / d.x;
            s1 = f.x / d.x;
            num_points = 2;
        }

        return num_points;
    }

    /* Left edge. */ {
        auto t = e.x / d.x;
        auto qy = p0.y + d.y * t;

        if (qy >= aabb_min.y && qy < aabb_max.y) {
            s0 = t;
            ++num_points;
        }
    }

    /* Right edge. */ {
        auto t = f.x / d.x;
        auto qy = p0.y + d.y * t;

        if (qy > aabb_min.y && qy <= aabb_max.y) {
            if (num_points == 0) {
                s0 = t;
                ++num_points;
            } else if (std::abs(t - s0) > EDYN_EPSILON) {
                s1 = t;
                ++num_points;
            }
        }
    }

    if (num_points == 2) {
        return num_points;
    }

    /* Bottom edge. */ {
        auto t = e.y / d.y;
        auto qx = p0.x + d.x * t;

        if (qx >= aabb_min.x && qx < aabb_max.x) {
            if (num_points == 0) {
                s0 = t;
                ++num_points;
            } else if (std::abs(t - s0) > EDYN_EPSILON) {
                s1 = t;
                ++num_points;
            }
        }
    }

    if (num_points == 2) {
        return num_points;
    }

    /* Top edge. */ {
        auto t = f.y / d.y;
        auto qx = p0.x + d.x * t;

        if (qx > aabb_min.x && qx <= aabb_max.x) {
            if (num_points == 0) {
                s0 = t;
                ++num_points;
            } else if (std::abs(t - s0) > EDYN_EPSILON) {
                s1 = t;
                ++num_points;
            }
        }
    }

    return num_points;
}

bool point_in_polygonal_prism(const std::vector<vector3> &vertices,
                              const std::vector<size_t> &indices,
                              const vector3 &normal, const vector3 &point) {
    const auto count = indices.size();
    EDYN_ASSERT(count > 2);

    for (size_t i = 0; i < count; ++i) {
        auto j = (i + 1) % count;
        auto idx0 = indices[i];
        auto idx1 = indices[j];
        auto &v0 = vertices[idx0];
        auto &v1 = vertices[idx1];
        auto d = v1 - v0;
        auto t = cross(d, normal);

        if (dot(point - v0, t) > EDYN_EPSILON) {
            return false;
        }
    }

    return true;
}

bool point_in_polygonal_prism(const std::vector<vector3> &vertices,
                              const vector3 &normal, const vector3 &point) {
    const auto count = vertices.size();
    EDYN_ASSERT(count > 2);

    for (size_t i = 0; i < count; ++i) {
        const auto j = (i + 1) % count;
        auto &v0 = vertices[i];
        auto &v1 = vertices[j];
        auto d = v1 - v0;
        auto t = cross(d, normal);

        if (dot(point - v0, t) > EDYN_EPSILON) {
            return false;
        }
    }

    return true;
}

// Reference: Real-Time Collision Detection - Christer Ericson,
// Section 5.3.3 - Intersecting Ray or Segment Against Box.
bool intersect_segment_aabb(vector3 p0, vector3 p1,
                            vector3 aabb_min, vector3 aabb_max) {
    auto aabb_center = (aabb_min + aabb_max) * scalar(0.5);
    auto half_extents = aabb_max - aabb_center;
    auto midpoint = (p0 + p1) * scalar(0.5);
    auto half_length = p1 - midpoint;
    midpoint -= aabb_center; // Translate box to origin.

    // Test coordinate axes.
    auto abs_half_length = abs(half_length);

    for (auto i = 0; i < 3; ++i) {
        if (std::abs(midpoint[i]) > half_extents[i] + abs_half_length[i]) {
            return false;
        }
    }

    // Add epsilon to better handle the parallel case.
    abs_half_length += vector3_one * EDYN_EPSILON;

    // Test cross products of segment direction with coordinate axes.
    if (std::abs(midpoint.y * half_length.z - midpoint.z * half_length.y) >
        half_extents.y * abs_half_length.z + half_extents.z * abs_half_length.y) {
        return false;
    }

    if (std::abs(midpoint.z * half_length.x - midpoint.x * half_length.z) >
        half_extents.z * abs_half_length.x + half_extents.x * abs_half_length.z) {
        return false;
    }

    if (std::abs(midpoint.x * half_length.y - midpoint.y * half_length.x) >
        half_extents.x * abs_half_length.y + half_extents.y * abs_half_length.x) {
        return false;
    }

    // No separating axis found. Segment intersects AABB.
    return true;
}

intersect_ray_cylinder_result intersect_ray_cylinder(vector3 p0, vector3 p1, vector3 pos, quaternion orn, scalar radius, scalar half_length, scalar &u) {
    // Let a plane be defined by the ray and the vector orthogonal to the
    // cylinder axis and the ray (i.e. their cross product). This plane cuts
    // the cylinder and their intersection is an ellipse with vertical half
    // length equals to the cylinder radius and horizontal half length bigger
    // than that. First, the parameters for the closest point between the lines
    // spanned by the cylinder axis and the ray are found. By subtracting an
    // amount from the parameter for the ray, the intersection point can be
    // found.
    auto cyl_dir = quaternion_x(orn);
    vector3 cyl_vertices[] = {
        pos + cyl_dir * half_length,
        pos - cyl_dir * half_length
    };
    scalar s, t;

    if (!closest_point_line_line(cyl_vertices[0], cyl_vertices[1], p0, p1, s, t)) {
        return {intersect_ray_cylinder_result::kind::parallel_directions};
    }

    auto radius_sqr = square(radius);
    auto closest_cyl = lerp(cyl_vertices[0], cyl_vertices[1], s);
    auto closest_ray = lerp(p0, p1, t);
    auto normal = closest_ray - closest_cyl;
    auto dist_sqr = length_sqr(normal);

    // Distance between lines bigger than radius.
    if (dist_sqr > radius_sqr) {
        return {intersect_ray_cylinder_result::kind::distance_greater_than_radius};
    }

    // Offset `t` backwards to place it where the intersection happens.
    auto d = p1 - p0;
    auto e = cyl_vertices[1] - cyl_vertices[0];
    auto dd = dot(d, d);
    auto ee = dot(e, e);
    auto de = dot(d, e);
    auto g_sqr = (radius_sqr - dist_sqr) * ee / (dd * ee - de * de);
    auto g = std::sqrt(g_sqr);
    u = t - g;
    return {intersect_ray_cylinder_result::kind::intersects, dist_sqr, normal};
}

bool intersect_ray_sphere(vector3 p0, vector3 p1, vector3 pos, scalar radius, scalar &t) {
    // Reference: Real-Time Collision Detection - Christer Ericson,
    // Section 5.3.2 - Intersecting Ray or Segment Against Sphere.
    // Substitute parametrized line function into sphere equation and
    // solve quadratic.
    auto d = p1 - p0;
    auto m = p0 - pos;
    auto a = dot(d, d);
    auto b = dot(m, d);
    auto c = dot(m, m) - radius * radius;

    // Exit if p0 is outside sphere and ray is pointing away from sphere.
    if (c > 0 && b > 0) {
        return false;
    }

    auto discr = b * b - a * c;

    // A negative discriminant corresponds to ray missing sphere.
    if (discr < 0) {
        return false;
    }

    // Ray intersects sphere. Compute smallest t.
    t = (-b - std::sqrt(discr)) / a;
    // If t is negative, ray started inside sphere so clamp it to zero.
    t = std::max(scalar(0), t);

    return true;
}

bool intersect_segment_triangle(const vector3 &p0, const vector3 &p1,
                                const std::array<vector3, 3> &vertices,
                                const vector3 &normal, scalar &t) {
    auto d = dot(p1 - p0, normal);
    auto e = dot(vertices[0] - p0, normal);
    t = 0;

    if (std::abs(d) > EDYN_EPSILON) {
        t = e / d;
    } else {
        // Ray is parallel to plane. Do not continue if ray is not
        // contained in plane.
        if (std::abs(e) > EDYN_EPSILON) {
            return false;
        }
    }

    if (t < scalar(0) || t > scalar(1)) {
        // Ray does not intersect plane.
        return false;
    }

    auto intersection = lerp(p0, p1, t);

    for (size_t i = 0; i < 3; ++i) {
        auto v0 = vertices[i];
        auto v1 = vertices[(i + 1) % 3];
        auto edge_dir = v1 - v0;
        auto tangent = cross(normal, edge_dir);

        if (dot(tangent, intersection - v0) < 0) {
            return false;
        }
    }

    return true;
}

}
