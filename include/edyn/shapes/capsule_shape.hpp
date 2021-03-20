#ifndef EDYN_SHAPES_CAPSULE_SHAPE_HPP
#define EDYN_SHAPES_CAPSULE_SHAPE_HPP

#include "edyn/comp/aabb.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/util/moment_of_inertia.hpp"

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
        return moment_of_inertia_solid_capsule(mass, half_length * 2, radius);
    }
};

}

#endif // EDYN_SHAPES_CAPSULE_SHAPE_HPP