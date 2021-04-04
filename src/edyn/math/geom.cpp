#include "edyn/math/geom.hpp"
#include "edyn/math/math.hpp"
#include "edyn/math/scalar.hpp"
#include "edyn/math/vector2_3_util.hpp"
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
                auto c1 = dot(d1, r1);
                auto a_inv = 1 / a;
                
                s   = clamp_unit(std::min(-c * a_inv, -c1 * a_inv));
                *sp = clamp_unit(std::max(-c * a_inv, -c1 * a_inv));

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
    vector3 &normal, scalar threshold) {

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
    size_t max_iterations = 3;

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
    vector3 sup_pos = support_point_circle(radiusB, posB_in_A, ornB_in_A, vector3_x);
    vector3 sup_neg = support_point_circle(radiusB, posB_in_A, ornB_in_A, -vector3_x);
    vector3 sup = std::abs(sup_pos.x) < std::abs(sup_neg.x) ? sup_pos : sup_neg;
    auto sup_in_B = to_object_space(sup, posB_in_A, ornB_in_A);
    auto initial_phi = std::atan2(sup_in_B.y, sup_in_B.z);

    // Newton-Raphson iterations.
    auto phi = initial_phi;
    size_t max_iterations = 10;

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

bool intersect_aabb(const vector3 &min0, const vector3 &max0,
                    const vector3 &min1, const vector3 &max1) {
    return (min0.x <= max1.x) &&
		   (max0.x >= min1.x) &&
		   (min0.y <= max1.y) &&
		   (max0.y >= min1.y) &&
		   (min0.z <= max1.z) &&
		   (max0.z >= min1.z);
}

vector3 support_point_circle(scalar radius, const vector3 &pos, const quaternion &orn, const vector3 &dir) {
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

    // Segments are parallel.
    if (perp_product(e, dp) < EDYN_EPSILON) {
        // Calculate intersection interval.
        auto dir = dot(dp, dq);
        auto denom_p = scalar(1) / dot(dp, dp);
        auto denom_q = scalar(1) / dot(dq, dq);

        if (dir > 0) {
            auto f = q1 - p1;
            s0 = dot(e, dp) * denom_p;
            t0 = -dot(e, dq) * denom_q;

            s1 = scalar(1) + dot(f, dp) * denom_p;
            t1 = scalar(1) - dot(f, dq) * denom_q;
        } else {
            auto f = q1 - p0;
            s0 = dot(f, dp) * denom_p;
            t0 = -dot(f, dq) * denom_q;

            auto g = q0 - p1;
            s1 = scalar(1) + dot(g, dp) * denom_p;
            t1 = scalar(1) - dot(g, dq) * denom_q;
        }

        if (s0 < 0 || s0 > 1 || t0 < 0 || t0 > 1 ||
            s1 < 0 || s1 > 1 || t1 < 0 || t1 > 1) {
            return 0;
        }
        
        return 2;
    }

    return 0;
}

scalar area_4_points(const vector3& p0, const vector3& p1, const vector3& p2, const vector3& p3) {
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

bool point_in_quad(const vector3 &p, 
                   const std::array<vector3, 4> &quad_vertices, 
                   const vector3 &quad_normal) {

    std::array<vector3, 4> quad_tangents;
    for (int i = 0; i < 4; ++i) {
        auto &v0 = quad_vertices[i];
        auto &v1 = quad_vertices[(i + 1) % 4];
        quad_tangents[i] = cross(quad_normal, v1 - v0);
    }

    scalar dots[4];
    for (int i = 0; i < 4; ++i) {
        dots[i] = dot(p - quad_vertices[i], quad_tangents[i]);
    }

    return dots[0] > -EDYN_EPSILON && dots[1] > -EDYN_EPSILON &&
           dots[2] > -EDYN_EPSILON && dots[3] > -EDYN_EPSILON;
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

        if (dot(point - v0, t) > 0) {
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

        if (dot(point - v0, t) > 0) {
            return false;
        }
    }

    return true;
}

}