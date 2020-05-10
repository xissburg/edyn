#include "edyn/math/geom.hpp"
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

size_t intersect_line_circle(scalar px, scalar py, 
                             scalar qx, scalar qy, 
                             scalar radius, 
                             scalar &s0, scalar &s1) {
    auto dx = qx - px, dy = qy - py;
    auto dl2 = dx * dx + dy * dy;
    auto dp = dx * px + dy * py;
    auto delta = dp * dp - dl2 * (px * px + py * py - radius * radius);

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
    const vector3 &cpos, const quaternion &corn,  scalar radius,
    const vector3 &p0, const vector3 &p1, size_t &num_points, 
    scalar &s0, vector3 &rc0, vector3 &rl0,
    scalar &s1, vector3 &rc1, vector3 &rl1, 
    vector3 &normal, scalar threshold) {

    // Line points in local disc space. The face of the disc points towards
    // the positive x-axis.
    auto corn_conj = conjugate(corn);
    auto q0 = to_object_space(p0, cpos, corn);
    auto q1 = to_object_space(p1, cpos, corn);
    auto qv = q1 - q0;
    auto qv_len_sqr = length_sqr(qv);
    auto qv_len = std::sqrt(qv_len_sqr);
    auto diameter = scalar(2) * radius;

    // If the projection of a segment of the line of length `diameter` on the x axis
    // is smaller than threshold, the line is considered to be parallel to the circle. 
    if (std::abs(qv.x) < EDYN_EPSILON) { // (std::abs(qv.x / qv_len) * diameter < threshold) {
        // Calculate line-circle intersection in the yz plane.
        // For the normal vector, calculate something orthogonal to the line.
        auto tangent = cross(qv, vector3_x); // tangent lies on the circle plane.
        normal = cross(qv, tangent);
        normal = rotate(corn, normal);
        normal = normalize(normal);

        num_points = intersect_line_circle(q0.y, q0.z, q1.y, q1.z, radius, s0, s1);

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
            closest_point_segment(p0, p1, cpos, s0, rl0);
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

            num_points = 1;
            return dl2;
        }
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

    // Newton-Raphson iterations.
    auto theta = initial_theta;
    size_t max_iterations = 3;

    for (size_t i = 0; i < max_iterations; ++i) {
        auto qv_len_sqr_inv = scalar(1) / qv_len_sqr;
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

    num_points = 1;

    return dist_sqr;
}

size_t intersect_circle_circle(scalar px, scalar py, 
                               scalar qx, scalar qy, 
                               scalar pr, scalar qr,
                               scalar &ix, scalar &iy,
                               scalar &jx, scalar &jy) {
    // Reference: Intersection of Linear and Circular Components in 2D - David Eberly
    // https://www.geometrictools.com/
    auto ux = px - qx, uy = py - qy;
    auto rsum = pr + qr;
    auto rsub = pr - qr;
    auto lu2 = ux * ux + uy * uy;

    if (lu2 <= EDYN_EPSILON) {
        return 0;
    }

    auto num = -(lu2 - rsum * rsum) * (lu2 - rsub * rsub);

    if (num < 0) {
        return 0;
    }

    auto lu2_inv = scalar(1) / lu2;
    auto denom_inv = lu2_inv * 0.25;
    auto t = std::sqrt(num * denom_inv);
    auto s = ((pr * pr - qr * qr) * lu2_inv + 1) * 0.5;

    auto vx = -uy, vy = ux;

    ix = px + s * ux + t * vx;
    iy = py + s * uy + t * vy;

    jx = px + s * ux - t * vx;
    jy = py + s * uy - t * vy;

    return t > EDYN_EPSILON ? 2 : 1;
}

scalar closest_point_circle_circle(
    const vector3 &posA, const quaternion &ornA, scalar radiusA,
    const vector3 &posB, const quaternion &ornB, scalar radiusB,
    size_t &num_points, closest_points_array &closest, 
    vector3 &normal) {

    auto normalA = rotate(ornA, vector3_x);
    auto normalB = rotate(ornB, vector3_x);

    auto ornA_conj = conjugate(ornA);
    auto ornB_conj = conjugate(ornB);

    // If discs have their normals in the same plane as their centers, the 
    // problem can be handled as segment-segment closest point.
    auto tangentA = cross(posA - posB, normalB);
    auto tangentB = cross(posA - posB, normalA);
    auto tangent = length_sqr(tangentA) > length_sqr(tangentB) ? tangentA : tangentB;

    if (std::abs(dot(tangent, normalA)) < 0.01 &&
        std::abs(dot(tangent, normalB)) < 0.01) {
        normal = normalB;

        auto bitangentA = cross(tangent, normalA);
        bitangentA -= normalA * dot(tangent, normalA);
        bitangentA = normalize(bitangentA);
        auto p0A = posA + bitangentA * radiusA;
        auto p1A = posA - bitangentA * radiusA;

        auto bitangentB = cross(tangent, normalB);
        bitangentB -= normalB * dot(tangent, normalB);
        bitangentB = normalize(bitangentB);
        auto p0B = posB + bitangentB * radiusB;
        auto p1B = posB - bitangentB * radiusB;

        scalar s0, s1, t0, t1;
        vector3 c0, c1, c2, c3;
        auto dist2 = closest_point_segment_segment(p0A, p1A, p0B, p1B, 
                                                   s0, t0, c0, c1, 
                                                   &num_points, 
                                                   &s1, &t1, &c2, &c3);
        size_t idx = 0;
        closest[idx].first = c0;
        closest[idx].second = c1;
        ++idx;

        if (num_points > 1) {
            closest[idx].first = c2;
            closest[idx].second = c3;
            ++idx;
        }

        if (std::abs(scalar(1) - dot(normalA, normalB)) < 0.01) {
            auto posBA = rotate(ornA_conj, posB - posA);
            auto np = intersect_circle_circle(0, 0,
                                              posBA.y, posBA.z,
                                              radiusA, radiusB,
                                              c0.y, c0.z,
                                              c1.y, c1.z);
            if (np > 0) {
                num_points += np;
                closest[idx].first = posA + rotate(ornA, vector3 {0, c0.y, c0.z});
                closest[idx].second = posA + rotate(ornA, vector3 {posBA.x , c0.y, c0.z});
                ++idx;

                if (np > 1) {
                    closest[idx].first = posA + rotate(ornA, vector3 {0, c1.y, c1.z});
                    closest[idx].second = posA + rotate(ornA, vector3 {posBA.x , c1.y, c1.z});
                    ++idx;
                }
            }
        }

        return dist2;
    }

    if (std::abs(scalar(1) - dot(normalA, normalB)) < 0.01) {
        auto posBA = rotate(ornA_conj, posB - posA);
        vector3 c0, c1;
        auto np = intersect_circle_circle(0, 0,
                                            posBA.y, posBA.z,
                                            radiusA, radiusB,
                                            c0.y, c0.z,
                                            c1.y, c1.z);
        if (np > 0) {
            num_points = np;
            closest[0].first = posA + rotate(ornA, vector3 {0, c0.y, c0.z});
            closest[0].second = posA + rotate(ornA, vector3 {posBA.x , c0.y, c0.z});

            if (np > 1) {
                closest[1].first = posA + rotate(ornA, vector3 {0, c1.y, c1.z});
                closest[1].second = posA + rotate(ornA, vector3 {posBA.x , c1.y, c1.z});
            }

            return posBA.x * posBA.x;
        }
    }

    // If the projection of the point on the disc closest to the plane of the
    // other disc is contained within the other disc, then that should be the
    // closest point.

    // Find closest point on disc A to plane of B.
    // B's normal in A's space.
    {
        auto normalBA = rotate(ornA_conj, normalB);
        // Squared length in yz plane.
        auto len2_normalBA_yz = normalBA.y * normalBA.y + normalBA.z * normalBA.z;
        auto closestA = vector3 {0, 0, radiusA};

        // No need to check if `len2_normalAB_yz` is zero since the discs are
        // deemed non-parallel at this point.
        auto normBA = radiusA / std::sqrt(len2_normalBA_yz);
        closestA.y = normalBA.y * normBA;
        closestA.z = normalBA.z * normBA;

        closestA = rotate(ornA, closestA);

        // Project on plane and check if distance is smaller than radius.
        auto distA = dot(closestA - posB, normalB);
        auto closestA_projB = closestA - normalB * distA;

        if (length_sqr(closestA_projB - posB) < radiusB) {
            num_points = 1;
            closest[0].first = closestA;
            closest[0].second = closestA_projB;
            normal = normalB;
            return distA * distA;
        }
    }

    // Find closest point on disc B to plane of A.
    // A's normal in B's space.
    {
        auto normalAB = rotate(ornB_conj, normalA);
        // Squared length in yz plane.
        auto len2_normalAB_yz = normalAB.y * normalAB.y + normalAB.z * normalAB.z;
        auto closestB = vector3 {0, 0, radiusB};

        // No need to check if `len2_normalAB_yz` is zero since the discs are
        // deemed non-parallel at this point.
        auto normAB = radiusA / std::sqrt(len2_normalAB_yz);
        closestB.y = normalAB.y * normAB;
        closestB.z = normalAB.z * normAB;

        closestB = rotate(ornB, closestB);

        // Project on plane and check if distance is smaller than radius.
        auto distB = dot(closestB - posA, normalA);
        auto closestB_projA = closestB - normalA * distB;

        if (length_sqr(closestB_projA - posA) < radiusA) {
            num_points = 1;
            closest[0].first = closestB;
            closest[0].second = closestB_projA;
            normal = -normalA;
            return distB * distB;
        }
    }

    // Transform disc A onto disc B's space.
    auto posAB = rotate(ornB_conj, posA - posB);

    // Build ortho basis on B (in A's space).
    auto ornAB = ornB_conj * ornA;
    auto u = rotate(ornAB, vector3_y) * radiusA;
    auto v = rotate(ornAB, vector3_z) * radiusA;

    scalar s = 0;
    scalar ds = pi / 18;
    auto dist2 = EDYN_SCALAR_MAX;

    for (;;) {
        auto q = posAB + u * std::cos(s) + v * std::sin(s);
        auto r = q;
        vector3 d;
        scalar l2;

        if (q.y * q.y + q.z * q.z > radiusB * radiusB) {
            r.x = 0;
            r = normalize(r) * radiusB;
            d = q - r;
            l2 = length_sqr(d);
        } else {
            r = {0, q.y, q.z};
            d = {q.x, 0, 0};
            l2 = q.x * q.x;
        }

        if (l2 < dist2) {
            dist2 = l2;
            closest[0].first = posB + rotate(ornB, q);
            closest[0].second = posB + rotate(ornB, r);
            normal = l2 > EDYN_EPSILON ? d / std::sqrt(l2) : vector3_x;
            //if (normal.x < 0) normal = -normal;
            normal = rotate(ornB, normal);
        }

        if (s >= 2 * pi) { break; }
        s = std::min(s + ds, 2 * pi);
    }

    num_points = 1;

    return dist2;
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

}