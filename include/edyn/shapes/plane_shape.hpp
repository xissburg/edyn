#ifndef EDYN_SHAPES_PLANE_SHAPE_HPP
#define EDYN_SHAPES_PLANE_SHAPE_HPP

#include "edyn/math/vector3.hpp"
#include "edyn/serialization/math_s11n.hpp"

namespace edyn {

/**
 * @brief A plane shape.
 * @remarks Planes can only be assigned to static rigid bodies.
 * Position and orientation are ignored during collision detection.
 */
struct plane_shape {
    vector3 normal;
    scalar constant;
};

template<typename Archive>
void serialize(Archive &archive, plane_shape &s) {
    archive(s.normal);
    archive(s.constant);
}

}

#endif // EDYN_SHAPES_PLANE_SHAPE_HPP
