#ifndef EDYN_MATH_GEOM_HPP
#define EDYN_MATH_GEOM_HPP

#include "constants.hpp"
#include "quaternion.hpp"
#include "vector2.hpp"
#include "edyn/util/array.hpp"

namespace edyn {
/**
 * Geometric utilities.
 */

/**
 * @brief Computes the point in the segment `q(t) = q0 + t*(q1 - q0)` closest
 * to point `p`.
 * 
 * @param q0 Initial point in segment.
 * @param q1 End point in segment.
 * @param p The point.
 * @param t Outputs the parameter where `q(t)` gives the closest point to `p`.
 * @param q Outputs the point in `q(t)` closest to `p`.
 * @return The squared distance between `q(t)` an `p`.
 */
scalar closest_point_segment(const vector3 &q0, const vector3 &q1,
                             const vector3 &p, scalar &t, vector3 &q);

scalar distance_sqr_line(const vector3 &q0, const vector3 &dir,
                         const vector3 &p);

/**
 * @brief Computes the point in the line `q(t) = q0 + t*dir` closest
 * to point `p`.
 * 
 * @param q0 Point in line.
 * @param dir Line direction vector.
 * @param p The point.
 * @param t Outputs the parameter where `q(t)` gives the closest point to `p`.
 * @param r Outputs the point in `q(t)` closest to `p`.
 * @return The squared distance between `q(t)` an `p`.
 */
scalar closest_point_line(const vector3 &q0, const vector3 &dir,
                          const vector3 &p, scalar &t, vector3 &r);

/**
 * @brief Computes the closest points `c1` and `c2` of segments 
 * `s1(s) = p1 + s*(q1 - p1)` and `s2(t) = p2 + t*(q2 - p2)`, 
 * where `0 <= s <= 1` and `0 <= t <= 1`.
 * 
 * @param p1 Initial point in the first segment.
 * @param q1 End point in the first segment.
 * @param p2 Initial point in the second segment.
 * @param q2 End point in the second segment.
 * @param s Outputs the parameter where `s1(s)` gives the closest point to `s2`.
 * @param t Outputs the parameter where `s2(t)` gives the closest point to `s1`.
 * @param c1 Outputs the point in `s1` closest to `s2`.
 * @param c2 Outputs the point in `s2` closest to `s1`.
 * @param num_points Optional pointer to store the number of closest points. If
 *        not `nullptr` and the segments are parallel, two closest points will 
 *        be generated if the projection of one segment onto the other is a range
 *        of points.
 * @param sp Outputs the parameter where `s1(s)` gives the closest point to `s2`
 *        if segments are parallel.
 * @param tp Outputs the parameter where `s2(t)` gives the closest point to `s1`
 *        if segments are parallel.
 * @param c1p Outputs the second point in `s1` closest to `s2` if segments are
 *        parallel.
 * @param c2p Outputs the second point in `s2` closest to `s1` if segments are
 *        parallel.
 * @return The squared distance between `s1(s)` and `s2(t)`.
 */
scalar closest_point_segment_segment(const vector3 &p1, const vector3 &q1, 
                                     const vector3 &p2, const vector3 &q2, 
                                     scalar &s, scalar &t, 
                                     vector3 &c1, vector3 &c2,
                                     size_t *num_points = nullptr,
                                     scalar *sp = nullptr, scalar *tp = nullptr, 
                                     vector3 *c1p = nullptr, vector3 *c2p = nullptr);

scalar closest_point_disc(const vector3 &dpos, const quaternion &dorn, scalar radius, 
                          const vector3 &p, vector3 &q);

/**
 * Computes the closest points between a line `p(s) = p0 + s*(p1 - p0)` and a circle.
 * @param cpos Center of circle.
 * @param corn Orientation of circle.The face of the circle points towards the
 *             positive x-axis.
 * @param radius Circle radius.
 * @param p0 A point in the line.
 * @param p1 Another point in the line.
 * @param num_points Number of closest points. Can be two in case the line is 
 *        nearly parallel to the plane that contains the circle and its 
 *        projection onto the plane intersects the circle.
 * @param s0 Outputs the parameter where the line gives the first closest point
 *        to the circle.
 * @param rc0 First closest point in the circle.
 * @param rl0 First closest point in the line.
 * @param s1 Outputs the parameter where the line gives the second closest
 *        point to the circle.
 * @param rc1 Second closest point in the circle.
 * @param rl1 Second closest point in the line.
 * @param normal Normal vector pointing out the circle.
 * @param threshold Value used to determine whether the line is parallel to circle.
 * @return The squared distance.
 */
scalar closest_point_circle_line(
    const vector3 &cpos, const quaternion &corn, scalar radius,
    const vector3 &p0, const vector3 &p1, size_t &num_points, 
    scalar &s0, vector3 &rc0, vector3 &rl0,
    scalar &s1, vector3 &rc1, vector3 &rl1, 
    vector3 &normal, scalar threshold = contact_breaking_threshold);

scalar closest_point_circle_circle(
    const vector3 &posA, const quaternion &ornA, scalar radiusA,
    const vector3 &posB, const quaternion &ornB, scalar radiusB,
    size_t &num_points, vector3 &rA0, vector3 &rB0, vector3 &rA1, vector3 &rB1, 
    vector3 &normal);

/**
 * Constructs an orthonomal basis given one vector. In other words, given a
 * plane normal, it finds two orthogonal vectors `p` and `q` which lie in
 * the plane.
 * @param n First basis vector, plane normal.
 * @param p Outputs the first vector on the plane.
 * @param q Outputs the second vector on the plane, orthogonal to `p`.
 */
void plane_space(const vector3 &n, vector3 &p, vector3 &q);

bool intersect_aabb(const vector3 &min0, const vector3 &max0,
                    const vector3 &min1, const vector3 &max1);

size_t intersect_segments(const vector2 &p0, const vector2 &p1,
                          const vector2 &q0, const vector2 &q1,
                          scalar &s0, scalar &t0,
                          scalar &s1, scalar &t1);

size_t intersect_line_circle(scalar px, scalar py, 
                             scalar qx, scalar qy, 
                             scalar radius, 
                             scalar &s0, scalar &s1);

size_t intersect_circle_circle(scalar px, scalar py, 
                               scalar qx, scalar qy, 
                               scalar pr, scalar qr,
                               scalar &ix, scalar &iy,
                               scalar &jx, scalar &jy);

vector3 support_point_circle(scalar radius, const vector3 &pos, 
                             const quaternion &orn, const vector3 &dir);

template<size_t N>
void support_point_vertices(const std::array<vector3, N> &vertices, 
                              const vector3 &dir, size_t &idx, scalar &proj) {
    proj = -EDYN_SCALAR_MAX;

    for (size_t i = 0; i < N; ++i) {
        auto d = dot(vertices[i], dir);

        if (d > proj) {
            proj = d;
            idx = i;
        }
    }
}

scalar area_4_points(const vector3& p0, const vector3& p1, const vector3& p2, const vector3& p3);

template<size_t N> inline
size_t insert_index(std::array<vector3, N> points,
                    std::array<scalar, N> depths,
                    size_t num_points,
                    const vector3 &new_point,
                    scalar new_point_depth) {
    EDYN_ASSERT(num_points <= N);

    // Look for nearby points.
    auto closest_idx = SIZE_MAX;
    auto closest_dist_sqr = EDYN_SCALAR_MAX;

    for (size_t i = 0; i < num_points; ++i) {
        auto dist_sqr = distance_sqr(new_point, points[i]);
        if (dist_sqr < closest_dist_sqr) {
            closest_dist_sqr = dist_sqr;
            closest_idx = i;
        }
    }
    
    if (closest_dist_sqr < contact_breaking_threshold * contact_breaking_threshold) {
        return closest_idx;
    }

    // Return the index after last to signal the insertion of a new point.
    if (num_points < N) {
        return num_points;
    }

    // Find deepest point and don't replace it.
    auto deepest_dist = new_point_depth;
    auto deepest_dist_idx = N;

    for (size_t i = 0; i < N; ++i) {
        if (depths[i] < deepest_dist) {
            deepest_dist = depths[i];
            deepest_dist_idx = i;
        }
    }

    // The approximate area when the i-th point is removed.
    auto areas = make_array<5>(scalar(0));

    // Do not calculate it for the deepest point.
    if (deepest_dist_idx != 0) {
        areas[0] = area_4_points(new_point, points[1], points[2], points[3]);
    } 
    if (deepest_dist_idx != 1) {
        areas[1] = area_4_points(new_point, points[0], points[2], points[3]);
    } 
    if (deepest_dist_idx != 2) {
        areas[2] = area_4_points(new_point, points[0], points[1], points[3]);
    } 
    if (deepest_dist_idx != 3) {
        areas[3] = area_4_points(new_point, points[0], points[1], points[2]);
    }
    if (deepest_dist_idx != 4) { // Area without the new point.
        areas[4] = area_4_points(points[0], points[1], points[2], points[3]);
    }

    auto max_area = scalar(0);
    auto max_area_idx = SIZE_MAX;
    for (size_t i = 0; i < areas.size(); ++i) {
        if (areas[i] > max_area) {
            max_area = areas[i];
            max_area_idx = i;
        }
    }

    if (max_area_idx < max_contacts) {
        return max_area_idx;
    }

    // Ignore new point because the contact set is better as it is.
    return N;
}

}

#endif // EDYN_MATH_GEOM_HPP