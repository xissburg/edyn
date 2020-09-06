#ifndef EDYN_SERIALIZATION_SHAPE_SPHERE_SHAPE_S11N_HPP
#define EDYN_SERIALIZATION_SHAPE_SPHERE_SHAPE_S11N_HPP

#include "edyn/shapes/sphere_shape.hpp"

namespace edyn {

template<typename Archive>
void serialize(Archive &archive, sphere_shape &s) {
    archive(s.radius);
}

}

#endif // EDYN_SERIALIZATION_SHAPE_SPHERE_SHAPE_S11N_HPP