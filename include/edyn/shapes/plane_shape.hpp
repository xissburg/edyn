#ifndef EDYN_SHAPES_PLANE_SHAPE_HPP
#define EDYN_SHAPES_PLANE_SHAPE_HPP

#include "edyn/comp/aabb.hpp"
#include "edyn/math/quaternion.hpp"

namespace edyn {

struct plane_shape {
    vector3 normal;
    scalar constant;

    AABB aabb(const vector3 &pos, const quaternion &orn) const {
        if (normal == vector3_x) {
            return {
                {-EDYN_SCALAR_MAX, -EDYN_SCALAR_MAX, -EDYN_SCALAR_MAX},
                {               0,  EDYN_SCALAR_MAX,  EDYN_SCALAR_MAX}
            };
        }
        if (normal == -vector3_x) {
            return {
                {               0, -EDYN_SCALAR_MAX, -EDYN_SCALAR_MAX},
                { EDYN_SCALAR_MAX,  EDYN_SCALAR_MAX,  EDYN_SCALAR_MAX}
            };
        }
        if (normal == vector3_y) {
            return {
                {-EDYN_SCALAR_MAX, -EDYN_SCALAR_MAX, -EDYN_SCALAR_MAX},
                { EDYN_SCALAR_MAX,                0,  EDYN_SCALAR_MAX}
            };
        }
        if (normal == -vector3_y) {
            return {
                {-EDYN_SCALAR_MAX,                0, -EDYN_SCALAR_MAX},
                { EDYN_SCALAR_MAX,  EDYN_SCALAR_MAX,  EDYN_SCALAR_MAX}
            };
        }
        if (normal == vector3_z) {
            return {
                {-EDYN_SCALAR_MAX, -EDYN_SCALAR_MAX, -EDYN_SCALAR_MAX},
                { EDYN_SCALAR_MAX,  EDYN_SCALAR_MAX,                0}
            };
        }
        if (normal == -vector3_z) {
            return {
                {-EDYN_SCALAR_MAX, -EDYN_SCALAR_MAX,                0},
                { EDYN_SCALAR_MAX,  EDYN_SCALAR_MAX,  EDYN_SCALAR_MAX}
            };
        }
        return {-vector3_max, vector3_max};
    }

    vector3 inertia(scalar mass) const {
        return vector3_max;
    }
};

}

#endif // EDYN_SHAPES_PLANE_SHAPE_HPP