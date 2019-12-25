#ifndef EDYN_SHAPES_CAPSULE_SHAPE_HPP
#define EDYN_SHAPES_CAPSULE_SHAPE_HPP

#include "edyn/comp/aabb.hpp"
#include "edyn/math/quaternion.hpp"

namespace edyn {

struct capsule_shape {
    scalar radius;
    scalar half_length;

    AABB aabb(const vector3 &pos, const quaternion &orn) const {
        auto v = rotate(orn, vector3_x * half_length);
        auto p0 = pos - v;
        auto p1 = pos + v;
        auto offset = vector3 {radius, radius, radius};
        return {min(p0, p1) - offset, max(p0, p1) + offset};
    }

    vector3 inertia(scalar mass) const {
        auto l = half_length * 2;
        auto xx = scalar(0.5) * mass * radius * radius;
        auto yy_zz = mass * (scalar(1) / scalar(12) * (scalar(3) * radius * radius + l * l) +
                     scalar(0.4) * radius * radius + 
                     scalar(0.375) * radius * l + 
                     scalar(0.25) * l * l);
        return {xx, yy_zz, yy_zz};
    }
};

}

#endif // EDYN_SHAPES_CAPSULE_SHAPE_HPP