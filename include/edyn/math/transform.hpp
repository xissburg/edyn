#ifndef EDYN_MATH_TRANSFORM_HPP
#define EDYN_MATH_TRANSFORM_HPP

#include "edyn/math/vector3.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/math/matrix3x3.hpp"

namespace edyn {

/**
 * @brief Converts a point in world space to object space.
 * @param p A point in world space.
 * @param pos Position in world space.
 * @param basis Rotation matrix in world space.
 * @return The point `p` in object space.
 */
inline
vector3 to_object_space(const vector3 &p, const vector3 &pos, const matrix3x3 &basis) {
    // Multiplying a vector by a matrix on the right is equivalent to multiplying
    // by the transpose of the matrix on the left, and the transpose of a rotation
    // matrix is its inverse.
    return (p - pos) * basis;
}

/**
 * @brief Converts a point in object space to world space.
 * @param p A point in object space.
 * @param pos Position in world space.
 * @param basis Rotation matrix in world space.
 * @return The point `p` in world space.
 */
inline
vector3 to_world_space(const vector3 &p, const vector3 &pos, const matrix3x3 &basis) {
    return pos + basis * p;
}

inline
vector3 to_object_space(const vector3 &p, const vector3 &pos, const quaternion &orn) {
    return rotate(conjugate(orn), p - pos);
}

inline
vector3 to_world_space(const vector3 &p, const vector3 &pos, const quaternion &orn) {
    return pos + rotate(orn, p);
}

}

#endif // EDYN_MATH_TRANSFORM_HPP
