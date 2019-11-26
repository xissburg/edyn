#ifndef EDYN_COLLISION_SHAPE_SPHERE_SHAPE_HPP
#define EDYN_COLLISION_SHAPE_SPHERE_SHAPE_HPP

#include "edyn/math/scalar.hpp"
#include "edyn/comp/aabb.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/collision/contact_manifold.hpp"

namespace edyn {

struct sphere_shape {
    scalar radius;

    AABB aabb(const vector3 &pos, const quaternion &orn) const {
        return {
            {pos.x - radius, pos.y - radius, pos.z - radius},
            {pos.x + radius, pos.y + radius, pos.z + radius}
        };
    }

    vector3 inertia(scalar mass) const {
        scalar s = 0.4 * mass * radius * radius;
        return {s, s, s};
    }
};

contact_manifold collide(const sphere_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const sphere_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold);

}

#endif // EDYN_COLLISION_SHAPE_SPHERE_SHAPE_HPP