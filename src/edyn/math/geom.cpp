#include "edyn/math/geom.hpp"
#include "edyn/math/coordinate_axis.hpp"
#include "edyn/math/math.hpp"
#include "edyn/math/vector2_3_util.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/math/transform.hpp"
#include "edyn/math/triangle.hpp"
#include <algorithm>

namespace edyn {

scalar closest_point_segment(const vector3 &q0, const vector3 &q1,
                             const vector3 &p, scalar &t, vector3 &q) noexcept {
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
                         const vector3 &p) noexcept {
    auto w = p - q0;
    auto a = dot(w, dir);
    auto b = dot(dir, dir);
    EDYN_ASSERT(b > EDYN_EPSILON);
    auto t = a / b;
    auto q = q0 + dir * t;
    return length_sqr(p - q);
}

scalar closest_point_line(const vector3 &q0, const vector3 &dir,
                          const vector3 &p, scalar &t, vector3 &r) noexcept {
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
                             scalar &s, scalar &t) noexcept {
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
                                     vector3 *c1p, vector3 *c2p) noexcept {
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
                          scalar radius, coordinate_axis axis,
                          const vector3 &p, vector3 &q) noexcept {
    // Project point onto disc's plane.
    const auto normal = coordinate_axis_vector(axis, dorn);
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
                             scalar radius, scalar &s0, scalar &s1) noexcept {
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
    const vector3 &cpos, const quaternion &corn, scalar radius, coordinate_axis axis,
    const vector3 &p0, const vector3 &p1, size_t &num_points,
    scalar &s0, vector3 &rc0, vector3 &rl0,
    scalar &s1, vector3 &rc1, vector3 &rl1,
    vector3 &normal, scalar threshold) noexcept {

    // Line points in local disc space. The face of the disc points towards
    // the positive x-axis.
    auto q0 = to_object_space(p0, cpos, corn);
    auto q1 = to_object_space(p1, cpos, corn);
    auto qv = q1 - q0;
    auto qv_len_sqr = length_sqr(qv);

    // The circle lies in one of the three coordinate planes in object space.
    // The circle axis is the coordinate axis normal to the plane it lies onto.
    // Index of vector element in the cirle's object space that represents the
    // circle axis followed by the indices of the elements of the axes
    // orthogonal to the circle axis, i.e. tangent to the circle plane.
    auto norm_idx = static_cast<std::underlying_type_t<coordinate_axis>>(axis);
    auto tan_idx0 = (norm_idx + 1) % 3;
    auto tan_idx1 = (norm_idx + 2) % 3;

    // Length of the segment projected onto the circle plane.
    auto qv_proj_len = length(vector2{qv[tan_idx0], qv[tan_idx1]});
    auto diameter = square(radius);

    // Line points projected onto circle plane.
    auto q0_proj = vector2{q0[tan_idx0], q0[tan_idx1]};
    auto q1_proj = vector2{q1[tan_idx0], q1[tan_idx1]};

    // If the projection of a segment of the line of length `diameter` on the circle axis
    // is smaller than threshold, the line is considered to be parallel to the circle.
    if (qv_proj_len > EDYN_EPSILON &&
        std::abs(qv[norm_idx] / qv_proj_len) * diameter < threshold) {
        // Calculate line-circle intersection in the yz plane.
        // For the normal vector, calculate something orthogonal to the line.
        auto tangent = cross(qv, coordinate_axis_vector(axis)); // tangent lies on the circle plane.
        normal = cross(qv, tangent);
        normal = rotate(corn, normal);
        normal = normalize(normal);
        EDYN_ASSERT(length_sqr(normal) > EDYN_EPSILON);

        num_points = intersect_line_circle(q0_proj, q1_proj, radius, s0, s1);

        if (num_points > 0) {
            auto rl0_local = q0 + qv * s0;
            auto rc0_local = rl0_local;
            rc0_local[norm_idx] = 0;

            // Transform to world space.
            rl0 = cpos + rotate(corn, rl0_local);
            rc0 = cpos + rotate(corn, rc0_local);

            // The distance is simply the value along the circle axis of the
            // resulting point in the line in circle space.
            auto dist2 = square(rl0_local[norm_idx]);

            if (num_points > 1) {
                auto rl1_local = q0 + qv * s1;
                auto rc1_local = rl1_local;
                rc1_local[norm_idx] = 0;
                rl1 = cpos + rotate(corn, rl1_local);
                rc1 = cpos + rotate(corn, rc1_local);

                dist2 = std::min(dist2, square(rl1_local[norm_idx]));
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
    if (length_sqr(q0_proj) <= EDYN_EPSILON && length_sqr(q1_proj) <= EDYN_EPSILON ) {
        num_points = 1;
        normal = axis == coordinate_axis::x ? quaternion_y(corn) :
                (axis == coordinate_axis::y ? quaternion_z(corn) : quaternion_x(corn));
        s0 = -q0[norm_idx] / qv[norm_idx];
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

    // Intersect line with the circle plane and use the angle of this point as the
    // initial value.
    const auto q_plane = q0 - (q0[norm_idx] / qv[norm_idx]) * qv;
    const auto initial_theta = std::atan2(q_plane[tan_idx0], q_plane[tan_idx1]);
    const auto qv_len_sqr_inv = scalar(1) / qv_len_sqr;

    // Newton-Raphson iterations.
    auto theta = initial_theta;
    size_t max_iterations = 20;

    for (size_t i = 0; i < max_iterations; ++i) {
        auto sin_theta = std::sin(theta);
        auto cos_theta = std::cos(theta);

        // Function q(θ) = [0, sin(θ) * r, cos(θ) * r] and its first and second
        // derivatives. That's when circle lies in the yz plane. Elements must
        // be swapped according to which coordinate plane the circle lies onto.
        vector3 q_theta, d_q_theta, dd_q_theta;

        q_theta[norm_idx] = 0;
        q_theta[tan_idx0] = sin_theta * radius;
        q_theta[tan_idx1] = cos_theta * radius;

        d_q_theta[norm_idx] = 0;
        d_q_theta[tan_idx0] = cos_theta * radius;
        d_q_theta[tan_idx1] = -sin_theta * radius;

        dd_q_theta[norm_idx] = 0;
        dd_q_theta[tan_idx0] = -sin_theta * radius;
        dd_q_theta[tan_idx1] = -cos_theta * radius;

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

    // Local position of first result point on circle.
    vector3 rc0_local;
    rc0_local[norm_idx] = 0;
    rc0_local[tan_idx0] = closest_sin_theta * radius;
    rc0_local[tan_idx1] = closest_cos_theta * radius;

    // Local position of first result point on line.
    vector3 rl0_local;
    auto dist_sqr = closest_point_line(q0, qv, rc0_local, s0, rl0_local);

    rc0 = cpos + rotate(corn, rc0_local);
    rl0 = cpos + rotate(corn, rl0_local);

    // Get tangent at θ and use cross product with line to calculate normal.
    vector3 tangent;
    tangent[norm_idx] = 0;
    tangent[tan_idx0] = closest_cos_theta;
    tangent[tan_idx1] = -closest_sin_theta;
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
        normal[norm_idx] = 0;
        normal[tan_idx0] = closest_sin_theta;
        normal[tan_idx1] = closest_cos_theta;
        normal = rotate(corn, normal);
    }

    EDYN_ASSERT(length_sqr(normal) > EDYN_EPSILON);

    num_points = 1;

    return dist_sqr;
}

size_t intersect_circle_circle(const vector2 &posA, scalar radiusA,
                               const vector2 &posB, scalar radiusB,
                               vector2 &res0, vector2 &res1) noexcept {
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
    const vector3 &posA, const quaternion &ornA, scalar radiusA, coordinate_axis axisA,
    const vector3 &posB, const quaternion &ornB, scalar radiusB, coordinate_axis axisB,
    size_t &num_points, vector3 &rA0, vector3 &rB0, vector3 &rA1, vector3 &rB1,
    vector3 &normal) {

    auto normalA = coordinate_axis_vector(axisA, ornA);
    auto normalB = coordinate_axis_vector(axisB, ornB);

    auto norm_idxA = static_cast<std::underlying_type_t<coordinate_axis>>(axisA);
    auto tan_idxA0 = (norm_idxA + 1) % 3;
    auto tan_idxA1 = (norm_idxA + 2) % 3;

    auto norm_idxB = static_cast<std::underlying_type_t<coordinate_axis>>(axisB);
    auto tan_idxB0 = (norm_idxB + 1) % 3;
    auto tan_idxB1 = (norm_idxB + 2) % 3;

    auto posB_in_A = to_object_space(posB, posA, ornA);

    // Check if parallel.
    if (!(length_sqr(cross(normalA, normalB)) > EDYN_EPSILON)) {
        normal = normalB;
        auto posB_in_A_proj = vector2{posB_in_A[tan_idxA0], posB_in_A[tan_idxA1]};

        // Circle A is in the origin.
        vector2 c0, c1;
        auto np = intersect_circle_circle(vector2_zero, radiusA, posB_in_A_proj, radiusB, c0, c1);

        if (np > 0) {
            num_points = np;

            vector3 rA0_local;
            rA0_local[norm_idxA] = 0;
            rA0_local[tan_idxA0] = c0.x;
            rA0_local[tan_idxA1] = c0.y;

            auto rB0_local = rA0_local;
            rB0_local[norm_idxA] = posB_in_A[norm_idxA];

            rA0 = to_world_space(rA0_local, posA, ornA);
            rB0 = to_world_space(rB0_local, posA, ornA);

            if (np > 1) {
                vector3 rA1_local;
                rA1_local[norm_idxA] = 0;
                rA1_local[tan_idxA0] = c1.x;
                rA1_local[tan_idxA1] = c1.y;

                auto rB1_local = rA1_local;
                rB1_local[norm_idxA] = posB_in_A[norm_idxA];

                rA1 = to_world_space(rA1_local, posA, ornA);
                rB1 = to_world_space(rB1_local, posA, ornA);
            }

            return square(posB_in_A[norm_idxA]);
        } else {
            // Circles do not intersect.
            // One circle could be contained within the other.
            // If the circles do not intersect and any point of one of them is
            // contained in the other, then the entire circle is contained in
            // the other.
            num_points = 1;
            // Points towards B.
            auto dir = posB_in_A_proj;
            auto dir_len_sqr = length_sqr(dir);

            // Vector tangent to plane of circle A.
            auto tanA = vector3_zero;
            tanA[tan_idxA0] = 1;

            if (dir_len_sqr > EDYN_EPSILON) {
                dir /= std::sqrt(dir_len_sqr);

                auto pointA = tanA * radiusA; // A point on circle A.
                auto pointB_in_A = posB_in_A + tanA * radiusB; // A point on circle B in A's space.
                // A contains B if the distance of a point in B to the origin
                // is smaller than the radius of A.
                auto A_contains_B = length_sqr(vector2{pointB_in_A[tan_idxA0], pointB_in_A[tan_idxA1]}) < radiusA * radiusA;
                // B contains A if the distance between a point in A and the
                // center of B is smaller than the radius of B.
                auto B_contains_A = distance_sqr(vector2{pointA[tan_idxA0], pointA[tan_idxA1]}, posB_in_A_proj) < radiusB * radiusB;

                // Direction vector pointing towards closest point from circle center.
                vector3 dirA, dirB;
                dirA[norm_idxA] = 0;
                dirA[tan_idxA0] = dir.x;
                dirA[tan_idxA1] = dir.y;

                dirB[norm_idxB] = 0;
                dirB[tan_idxB0] = dir.x;
                dirB[tan_idxB1] = dir.y;

                // Reverse it based on which region the circles are located so they
                // point in the right direction, towards closest point.
                dirA *= B_contains_A ? -1 : 1;
                dirB *= B_contains_A || (!A_contains_B && !B_contains_A) ? -1 : 1;

                rA0 = to_world_space(dirA * radiusA, posA, ornA);
                rB0 = to_world_space(posB_in_A + dirB * radiusB, posA, ornA);
                return distance_sqr(rA0, rB0);
            } else {
                // Concentric. Return a pair of points that lie on the same line
                // crossing the center of the circles, which are both at the
                // origin in this case.
                rA0 = to_world_space(tanA * radiusA, posA, ornA);
                rB0 = to_world_space(tanA * radiusB, posA, ornA);
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

    // Build ortho basis of B (in A's space).
    auto ornB_in_A = conjugate(ornA) * ornB;
    vector3 u, v;

    switch (axisA) {
    case coordinate_axis::x:
        u = quaternion_z(ornB_in_A);
        v = quaternion_y(ornB_in_A);
        break;
    case coordinate_axis::y:
        u = quaternion_x(ornB_in_A);
        v = quaternion_z(ornB_in_A);
        break;
    case coordinate_axis::z:
        u = quaternion_y(ornB_in_A);
        v = quaternion_x(ornB_in_A);
        break;
    }

    // Use angle of support point of B closest on A's plane as initial angle.
    vector3 sup_pos = support_point_circle(posB_in_A, ornB_in_A, radiusB, axisB, coordinate_axis_vector(axisA));
    vector3 sup_neg = support_point_circle(posB_in_A, ornB_in_A, radiusB, axisB, -coordinate_axis_vector(axisA));
    vector3 sup = std::abs(sup_pos[norm_idxA]) < std::abs(sup_neg[norm_idxA]) ? sup_pos : sup_neg;
    auto sup_in_B = to_object_space(sup, posB_in_A, ornB_in_A);
    auto initial_phi = std::atan2(sup_in_B[tan_idxA0], sup_in_B[tan_idxA1]);

    // Newton-Raphson iterations.
    auto phi = initial_phi;
    size_t max_iterations = 20;

    for (size_t i = 0; i < max_iterations; ++i) {
        auto cos_phi = std::cos(phi);
        auto sin_phi = std::sin(phi);

        auto p_phi = posB_in_A + (u * cos_phi + v * sin_phi) * radiusB;
        auto d_p_phi = (u * -sin_phi + v * cos_phi) * radiusB;
        auto dd_p_phi = (u * -cos_phi + v * -sin_phi) * radiusB;

        auto theta = std::atan2(p_phi[tan_idxA0], p_phi[tan_idxA1]);
        auto cos_theta = std::cos(theta);
        auto sin_theta = std::sin(theta);

        vector3 q_theta, d_q_theta, dd_q_theta;

        q_theta[norm_idxA] = 0;
        q_theta[tan_idxA0] = sin_theta * radiusA;
        q_theta[tan_idxA1] = cos_theta * radiusA;

        d_q_theta[norm_idxA] = 0;
        d_q_theta[tan_idxA0] = cos_theta * radiusA;
        d_q_theta[tan_idxA1] = -sin_theta * radiusA;

        dd_q_theta[norm_idxA] = 0;
        dd_q_theta[tan_idxA0] = -sin_theta * radiusA;
        dd_q_theta[tan_idxA1] = -cos_theta * radiusA;

        // Function d(φ) is the vector connecting the closest points.
        // First and second derivates follow.
        auto d_phi = p_phi - q_theta;
        auto d_d_phi = d_p_phi - d_q_theta;
        auto dd_d_phi = dd_p_phi - dd_q_theta;

        // Function f(φ) = d · d / 2 gives us a scalar proportional to the
        // length of d(φ).
        // First derivative f' = d'· d
        // Second derivative f" = d'· d + d"· d
        // auto f_phi = scalar(0.5) * dot(d_phi, d_phi);
        auto d_f_phi = dot(d_phi, d_d_phi);
        auto dd_f_phi = dot(d_d_phi, d_d_phi) + dot(dd_d_phi, d_phi);

        EDYN_ASSERT(dd_f_phi > 0 || dd_f_phi < 0);

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

    auto theta = std::atan2(rB0[tan_idxA0], rB0[tan_idxA1]);
    auto cos_theta = std::cos(theta);
    auto sin_theta = std::sin(theta);
    rA0[norm_idxA] = 0;
    rA0[tan_idxA0] = sin_theta * radiusA;
    rA0[tan_idxA1] = cos_theta * radiusA;

    rA0 = to_world_space(rA0, posA, ornA);
    rB0 = to_world_space(rB0, posA, ornA);

    auto dir = rA0 - rB0;
    auto dist_sqr = length_sqr(dir);

    // Get tangents and use cross product to calculate normal.
    vector3 tangentA;
    tangentA[norm_idxA] = 0;
    tangentA[tan_idxA0] = cos_theta;
    tangentA[tan_idxA1] = -sin_theta;

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
        normal[norm_idxA] = 0;
        normal[tan_idxA0] = sin_theta;
        normal[tan_idxA1] = cos_theta;
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
                             scalar radius, coordinate_axis axis,
                             const vector3 &dir) noexcept {
    auto norm_idx = static_cast<std::underlying_type_t<coordinate_axis>>(axis);
    auto tan_idx0 = (norm_idx + 1) % 3;
    auto tan_idx1 = (norm_idx + 2) % 3;

    auto local_dir = rotate(conjugate(orn), dir);

    // Squared length in circle plane.
    auto len_plane_sqr = local_dir[tan_idx0] * local_dir[tan_idx0] +
                         local_dir[tan_idx1] * local_dir[tan_idx1];
    vector3 sup;

    if (len_plane_sqr > EDYN_EPSILON) {
        auto d = radius / std::sqrt(len_plane_sqr);
        sup[norm_idx] = 0;
        sup[tan_idx0] = local_dir[tan_idx0] * d;
        sup[tan_idx1] = local_dir[tan_idx1] * d;
    } else {
        sup[norm_idx] = 0;
        sup[tan_idx0] = radius;
        sup[tan_idx1] = 0;
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

static
scalar manifold_score(const vector3 &p0, const vector3 &p1, const vector3 &p2, const vector3 &p3) noexcept {
    // Calculate a value proportional to the surface area of the tetrahedron
    // with vertices (p0, p1, p2, p3).
    vector3 c0 = cross(p0 - p1, p0 - p2);
    vector3 c1 = cross(p0 - p2, p0 - p3);
    vector3 c2 = cross(p0 - p3, p0 - p1);
    vector3 c3 = cross(p1 - p2, p2 - p3);
    return length_sqr(c0) + length_sqr(c1) + length_sqr(c2) + length_sqr(c3);
}

insertion_point_result insertion_point_index(const vector3 *points,
                                             const scalar *depths,
                                             size_t count,
                                             size_t &num_points,
                                             const vector3 &new_point) noexcept {
    EDYN_ASSERT(num_points <= count);
    const auto max_dist_similar_sqr = contact_merging_threshold * contact_merging_threshold;

    if (num_points == 0) {
        return {point_insertion_type::append, num_points++};
    }

    if (num_points == 1) {
        // Replace if too close.
        if (distance_sqr(new_point, points[0]) > max_dist_similar_sqr) {
            return {point_insertion_type::append, num_points++};
        } else {
            return {point_insertion_type::similar, 0};
        }
    }

    if (num_points == 2) {
        // Check if collinear.
        if (length_sqr(cross(new_point - points[0], new_point - points[1])) > EDYN_EPSILON) {
            return {point_insertion_type::append, num_points++};
        } else {
            // Points are collinear. Select a point to replace.
            // Maximize segment length.
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
            // Check if point is in triangle.
            if (std::abs(dot(new_point - points[0], normal)) < EDYN_EPSILON &&
                point_in_triangle(vertices, normal, new_point)) {
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

            // Points coincide. Find the pair of points and replace one of them.
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

    // Select the combination of points with the best score.
    auto scores = make_array<4>(scalar(0));
    scores[0] = manifold_score(new_point, points[1], points[2], points[3]);
    scores[1] = manifold_score(new_point, points[0], points[2], points[3]);
    scores[2] = manifold_score(new_point, points[0], points[1], points[3]);
    scores[3] = manifold_score(new_point, points[0], points[1], points[2]);

    auto current_score = manifold_score(points[0], points[1], points[2], points[3]);
    auto max_score = current_score;
    auto max_score_idx = SIZE_MAX;

    for (size_t i = 0; i < scores.size(); ++i) {
        if (scores[i] > max_score) {
            max_score = scores[i];
            max_score_idx = i;
        }
    }

    if (max_score_idx < max_contacts) {
        auto type = distance_sqr(points[max_score_idx], new_point) < max_dist_similar_sqr ?
                    point_insertion_type::similar : point_insertion_type::replace;
        return {type, max_score_idx};
    }

    // Ignore new point because the current contact set is better as it is.
    return {point_insertion_type::none, count};
}

vector3 closest_point_box_outside(const vector3 &half_extent, const vector3 &p) noexcept {
    auto closest = p;
    closest.x = std::min(half_extent.x, closest.x);
    closest.x = std::max(-half_extent.x, closest.x);
    closest.y = std::min(half_extent.y, closest.y);
    closest.y = std::max(-half_extent.y, closest.y);
    closest.z = std::min(half_extent.z, closest.z);
    closest.z = std::max(-half_extent.z, closest.z);
    return closest;
}

scalar closest_point_box_inside(const vector3 &half_extent, const vector3 &p, vector3 &closest, vector3 &normal) noexcept {
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
                           scalar &s0, scalar &s1) noexcept {
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
                              const vector3 &normal, const vector3 &point) noexcept {
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
                            vector3 aabb_min, vector3 aabb_max) noexcept {
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

intersect_ray_cylinder_result intersect_ray_cylinder(vector3 p0, vector3 p1,
                                                     vector3 pos, quaternion orn,
                                                     scalar radius, scalar half_length,
                                                     coordinate_axis axis,
                                                     scalar &fraction_in, scalar &fraction_out) noexcept {
    // Let a plane be defined by the ray and the vector orthogonal to the
    // cylinder axis and the ray (i.e. their cross product). This plane cuts
    // the cylinder and their intersection is an ellipse with vertical half
    // length (aka semi-minor axis) equals to the cylinder radius and horizontal
    // half length (aka semi-major axis) greater than or equal to the radius.
    // The distance between the ray and the cylinder axis gives us the value of
    // `y` in the ellipse equation `x^2/a^2 + y^2/b^2 = 1`. Solving for `x` will
    // give us information to calculate the exact point where the ray intersects
    // the cylinder.
    // Detailed derivation: https://xissburg.github.io/2022-10-03-intersecting-line-against-cylinder

    auto cyl_dir = coordinate_axis_vector(axis, orn);
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
    auto delta_sqr = (radius_sqr - dist_sqr) * ee / (dd * ee - de * de);
    auto delta = std::sqrt(delta_sqr);
    fraction_in = t - delta;
    fraction_out = t + delta;
    return {intersect_ray_cylinder_result::kind::intersects, dist_sqr, normal};
}

bool intersect_ray_sphere(vector3 p0, vector3 p1, vector3 pos, scalar radius, scalar &t) noexcept {
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
                                const vector3 &normal, scalar &t) noexcept {
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

bool edges_generate_minkowski_face(vector3 A, vector3 B, vector3 C_neg, vector3 D_neg, vector3 B_x_A, vector3 D_x_C) {
    auto CBA = -dot(C_neg, B_x_A);
    auto DBA = -dot(D_neg, B_x_A);
    auto ADC = dot(A, D_x_C);
    auto BDC = dot(B, D_x_C);

    return CBA * DBA < 0 && ADC * BDC < 0 && CBA * BDC > 0;
}

}
