#ifndef EDYN_MATH_GEOM_HPP
#define EDYN_MATH_GEOM_HPP

#include "constants.hpp"
#include "quaternion.hpp"
#include <array>

namespace edyn {
/**
 * Geometric utilities.
 */

using triangle_vertices = std::array<vector3, 3>;
using triangle_edges = std::array<vector3, 3>;

inline
triangle_edges get_triangle_edges(const triangle_vertices &vertices) {
    return {
        vertices[1] - vertices[0],
        vertices[2] - vertices[1],
        vertices[0] - vertices[2]
    };
}

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
 * Computes the closest points between a line `p(s) = p0 + s*(p1 - p0)` and a disc.
 * @param cpos Center of disc.
 * @param corn Orientation of disc.The face of the disc points towards the
 *             positive x-axis.
 * @param radius Disc radius.
 * @param p0 A point in the line.
 * @param p1 Another point in the line.
 * @param num_points Number of closest points. Can be two in case the line is 
 *        nearly parallel to the plane that contains the disc and its 
 *        projection onto the plane intersects the disc.
 * @param s0 Outputs the parameter where the line gives the first closest point
 *        to the disc.
 * @param cc0 First closest point in the disc.
 * @param cl0 First closest point in the line.
 * @param s1 Outputs the parameter where the line gives the second closest
 *        point to the disc.
 * @param cc1 Second closest point in the disc.
 * @param cl1 Second closest point in the line.
 * @param normal Normal vector pointing out the disc.
 * @param threshold Value used to determine whether the line is parallel to disc.
 * @return The squared distance.
 */
scalar closest_point_disc_line(const vector3 &cpos, const quaternion &corn, scalar radius,
                               const vector3 &p0, const vector3 &p1, size_t &num_points, 
                               scalar &s0, vector3 &cc0, vector3 &cl0,
                               scalar &s1, vector3 &cc1, vector3 &cl1, 
                               vector3 &normal, scalar threshold = contact_breaking_threshold);

using closest_points_array = std::array<std::pair<vector3, vector3>, max_contacts>;

scalar closest_point_disc_disc(const vector3 &posA, const quaternion &ornA, scalar radiusA,
                               const vector3 &posB, const quaternion &ornB, scalar radiusB,
                               size_t &num_points, closest_points_array &, 
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

/**
 * Checks whether point `p` is contained within the infinite prism with 
 * triangular base defined by the given `vertices` and direction `normal`.
 */
bool point_in_triangle(const triangle_vertices &, 
                       const vector3 &normal, 
                       const vector3 &p);

bool intersect_aabb(const vector3 &min0, const vector3 &max0,
                    const vector3 &min1, const vector3 &max1);

size_t intersect_line_circle(scalar px, scalar py, 
                             scalar qx, scalar qy, 
                             scalar radius, 
                             scalar &s0, scalar &s1);

}

#endif // EDYN_MATH_GEOM_HPP