#ifndef EDYN_COLLISION_SHAPE_SPHERE_SHAPE_HPP
#define EDYN_COLLISION_SHAPE_SPHERE_SHAPE_HPP

#include "edyn/math/scalar.hpp"
#include "edyn/comp/aabb.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/collision/collision_result.hpp"
#include "edyn/util/moment_of_inertia.hpp"

namespace edyn {

struct sphere_shape {
    scalar radius;

    AABB aabb(const vector3 &pos, const quaternion &orn) const {
        return {
            {pos.x - radius, pos.y - radius, pos.z - radius},
            {pos.x + radius, pos.y + radius, pos.z + radius}
        };
    }

    matrix3x3 inertia(scalar mass) const {
        return diagonal_matrix(moment_of_inertia_solid_sphere(mass, radius));
    }
};

}

#endif // EDYN_COLLISION_SHAPE_SPHERE_SHAPE_HPP