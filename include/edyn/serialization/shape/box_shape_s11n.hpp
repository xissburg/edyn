#ifndef EDYN_SERIALIZATION_SHAPE_BOX_SHAPE_S11N_HPP
#define EDYN_SERIALIZATION_SHAPE_BOX_SHAPE_S11N_HPP

#include "edyn/shapes/box_shape.hpp"
#include "edyn/serialization/s11n_util.hpp"

namespace edyn {

template<typename Archive>
void serialize(Archive &archive, box_shape &s) {
    archive(s.half_extents);
}

template<typename Archive>
void serialize(Archive &archive, box_feature &f) {
    serialize_enum(archive, f);
}

}

#endif // EDYN_SERIALIZATION_SHAPE_BOX_SHAPE_S11N_HPP