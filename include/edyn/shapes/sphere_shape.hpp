#ifndef EDYN_COLLISION_SHAPE_SPHERE_SHAPE_HPP
#define EDYN_COLLISION_SHAPE_SPHERE_SHAPE_HPP

#include "edyn/math/scalar.hpp"
#include "edyn/comp/aabb.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/collision/collision_result.hpp"

namespace edyn {

struct sphere_shape {
    scalar radius;
};

}

#endif // EDYN_COLLISION_SHAPE_SPHERE_SHAPE_HPP