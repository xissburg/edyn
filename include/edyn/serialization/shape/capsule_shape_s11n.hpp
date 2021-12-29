#ifndef EDYN_SERIALIZATION_SHAPE_CAPSULE_SHAPE_S11N_HPP
#define EDYN_SERIALIZATION_SHAPE_CAPSULE_SHAPE_S11N_HPP

#include "edyn/shapes/capsule_shape.hpp"

namespace edyn {

template<typename Archive>
void serialize(Archive &archive, capsule_shape &s) {
    archive(s.half_length);
    archive(s.radius);
}

template<typename Archive>
void serialize(Archive &archive, capsule_feature &f) {
    serialize_enum(archive, f);
}

}

#endif // EDYN_SERIALIZATION_SHAPE_CAPSULE_SHAPE_S11N_HPP