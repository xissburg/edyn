#ifndef EDYN_SHAPES_PLANE_SHAPE_HPP
#define EDYN_SHAPES_PLANE_SHAPE_HPP

#include "edyn/comp/aabb.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/math/matrix3x3.hpp"

namespace edyn {

struct plane_shape {
    vector3 normal;
    scalar constant;
    static constexpr scalar aabb_half_extent = 99999;

    AABB aabb(const vector3 &pos, const quaternion &orn) const {
        vector3 unit_min, unit_max;

        if (normal == vector3_x) {
            unit_min = {-1, -1, -1};
            unit_max = { 0,  1,  1};
        } else if (normal == -vector3_x) {
            unit_min = { 0, -1, -1};
            unit_max = { 1,  1,  1};
        } else if (normal == vector3_y) {
            unit_min = {-1, -1, -1};
            unit_max = { 1,  0,  1};
        } else if (normal == -vector3_y) {
            unit_min = {-1,  0, -1};
            unit_max = { 1,  1,  1};
        } else if (normal == vector3_z) {
            unit_min = {-1, -1, -1};
            unit_max = { 1,  1,  0};
        } else if (normal == -vector3_z) {
            unit_min = {-1, -1,  0};
            unit_max = { 1,  1,  1};
        } else {
            unit_min = { 1 , 1 , 1};
            unit_max = {-1, -1, -1};
        }
        
        return {unit_min * aabb_half_extent + pos, 
                unit_max * aabb_half_extent + pos};
    }

    matrix3x3 inertia(scalar mass) const {
        return diagonal_matrix(vector3_max);
    }
};

}

#endif // EDYN_SHAPES_PLANE_SHAPE_HPP