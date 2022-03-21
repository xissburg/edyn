#ifndef EDYN_COLLISION_SHAPE_SPHERE_SHAPE_HPP
#define EDYN_COLLISION_SHAPE_SPHERE_SHAPE_HPP

#include "edyn/math/scalar.hpp"

namespace edyn {

struct sphere_shape {
    scalar radius;
};

template<typename Archive>
void serialize(Archive &archive, sphere_shape &s) {
    archive(s.radius);
}

}

#endif // EDYN_COLLISION_SHAPE_SPHERE_SHAPE_HPP
