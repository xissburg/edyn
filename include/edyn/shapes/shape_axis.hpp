#ifndef EDYN_SHAPES_SHAPE_AXIS_HPP
#define EDYN_SHAPES_SHAPE_AXIS_HPP

#include "edyn/math/quaternion.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/serialization/s11n_util.hpp"

namespace edyn {

enum class shape_axis : unsigned char {
    x, y, z
};

inline vector3 shape_axis_vector(shape_axis axis) {
    switch (axis) {
    case shape_axis::x:
        return vector3_x;
    case shape_axis::y:
        return vector3_y;
    case shape_axis::z:
        return vector3_z;
    }
}

inline vector3 shape_axis_vector(shape_axis axis, const quaternion &orn) {
    return rotate(orn, shape_axis_vector(axis));
}

template<typename Archive>
void serialize(Archive &archive, shape_axis &axis) {
    serialize_enum(archive, axis);
}

}

#endif // EDYN_SHAPES_SHAPE_AXIS_HPP
