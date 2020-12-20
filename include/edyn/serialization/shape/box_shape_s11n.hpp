#ifndef EDYN_SERIALIZATION_SHAPE_BOX_SHAPE_S11N_HPP
#define EDYN_SERIALIZATION_SHAPE_BOX_SHAPE_S11N_HPP

#include "edyn/shapes/box_shape.hpp"

namespace edyn {

template<typename Archive>
void serialize(Archive &archive, box_shape &s) {
    archive(s.half_extents);
}

}

#endif // EDYN_SERIALIZATION_SHAPE_BOX_SHAPE_S11N_HPP