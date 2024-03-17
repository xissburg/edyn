#ifndef EDYN_SHAPES_COORDINATE_AXIS_HPP
#define EDYN_SHAPES_COORDINATE_AXIS_HPP

#include "edyn/config/config.h"
#include "edyn/math/quaternion.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/serialization/s11n_util.hpp"

namespace edyn {

/**
 * @brief One of the three coordinate axes: X, Y, Z.
 */
enum class coordinate_axis : unsigned char {
    x, y, z
};

/**
 * @brief Vector for a coordinate axis.
 * @param axis Axis.
 * @return Vector.
 */
constexpr vector3 coordinate_axis_vector(coordinate_axis axis) {
    switch (axis) {
    case coordinate_axis::x:
        return vector3_x;
    case coordinate_axis::y:
        return vector3_y;
    case coordinate_axis::z:
        return vector3_z;
    }
    EDYN_ASSERT(false);
    return vector3_zero;
}

/**
 * @brief Rotated vector for a coordinate axis.
 * @param axis Axis.
 * @param q Rotation.
 * @return Rotated vector.
 */
constexpr vector3 coordinate_axis_vector(coordinate_axis axis, const quaternion &q) {
    return rotate(q, coordinate_axis_vector(axis));
}

template<typename Archive>
void serialize(Archive &archive, coordinate_axis &axis) {
    serialize_enum(archive, axis);
}

}

#endif // EDYN_SHAPES_COORDINATE_AXIS_HPP
