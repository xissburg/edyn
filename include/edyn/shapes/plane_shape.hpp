#ifndef EDYN_SHAPES_PLANE_SHAPE_HPP
#define EDYN_SHAPES_PLANE_SHAPE_HPP

#include "edyn/comp/aabb.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/math/matrix3x3.hpp"

namespace edyn {

struct plane_shape {
    vector3 normal;
    scalar constant;
};

}

#endif // EDYN_SHAPES_PLANE_SHAPE_HPP