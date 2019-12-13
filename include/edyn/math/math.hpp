#ifndef EDYN_MATH_MATH_HPP
#define EDYN_MATH_MATH_HPP

#include "constants.hpp"

namespace edyn {

/**
 * @return Value of `radians` converted to degrees.
 */
inline scalar to_degrees(scalar radians) {
    return radians / pi * 180;
}

/**
 * @return Value of `degress` converted to radians.
 */
inline scalar to_radians(scalar degrees) {
    return degrees / 180 * pi;
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
 * @return The squared distance between `s1(s)` and `s2(t)`.
 */
scalar closest_point_segment_segment(const vector3 &p1, const vector3 &q1, 
                                     const vector3 &p2, const vector3 &q2, 
                                     scalar &s, scalar &t, 
                                     vector3 &c1, vector3 &c2);

}

#endif // EDYN_MATH_MATH_HPP