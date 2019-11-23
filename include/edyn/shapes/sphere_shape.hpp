#ifndef EDYN_COLLISION_SHAPE_SPHERE_SHAPE_HPP
#define EDYN_COLLISION_SHAPE_SPHERE_SHAPE_HPP

#include "edyn/math/scalar.hpp"
#include "edyn/comp/aabb.hpp"
#include "edyn/math/quaternion.hpp"

namespace edyn {

struct sphere_shape {
    scalar radius;

    AABB aabb(const vector3 &pos, const quaternion &orn) const {
        return {
            {pos.x - radius, pos.y - radius, pos.z - radius},
            {pos.x + radius, pos.y + radius, pos.z + radius}
        };
    }
};

}

#endif // EDYN_COLLISION_SHAPE_SPHERE_SHAPE_HPP