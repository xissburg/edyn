#ifndef EDYN_SERIALIZATION_SHAPE_PLANE_SHAPE_S11N_HPP
#define EDYN_SERIALIZATION_SHAPE_PLANE_SHAPE_S11N_HPP

#include "edyn/shapes/plane_shape.hpp"

namespace edyn {

template<typename Archive>
void serialize(Archive &archive, plane_shape &s) {
    archive(s.normal);
    archive(s.constant);
}

}

#endif // EDYN_SERIALIZATION_SHAPE_PLANE_SHAPE_S11N_HPP