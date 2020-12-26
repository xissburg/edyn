#ifndef EDYN_SHAPES_PLANE_SHAPE_HPP
#define EDYN_SHAPES_PLANE_SHAPE_HPP

#include "edyn/comp/aabb.hpp"
#include "edyn/math/quaternion.hpp"

namespace edyn {

struct plane_shape {
    vector3 normal;
    scalar constant;
    static constexpr scalar aabb_size = 99999;

    AABB aabb(const vector3 &pos, const quaternion &orn) const {
        if (normal == vector3_x) {
            return {
                {-aabb_size, -aabb_size, -aabb_size},
                {         0,  aabb_size,  aabb_size}
            };
        }
        if (normal == -vector3_x) {
            return {
                {         0, -aabb_size, -aabb_size},
                { aabb_size,  aabb_size,  aabb_size}
            };
        }
        if (normal == vector3_y) {
            return {
                {-aabb_size, -aabb_size, -aabb_size},
                { aabb_size,          0,  aabb_size}
            };
        }
        if (normal == -vector3_y) {
            return {
                {-aabb_size,          0, -aabb_size},
                { aabb_size,  aabb_size,  aabb_size}
            };
        }
        if (normal == vector3_z) {
            return {
                {-aabb_size, -aabb_size, -aabb_size},
                { aabb_size,  aabb_size,          0}
            };
        }
        if (normal == -vector3_z) {
            return {
                {-aabb_size, -aabb_size,          0},
                { aabb_size,  aabb_size,  aabb_size}
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