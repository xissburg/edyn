#ifndef EDYN_SERIALIZATION_SHAPE_PAGED_MESH_SHAPE_S11N_HPP
#define EDYN_SERIALIZATION_SHAPE_PAGED_MESH_SHAPE_S11N_HPP

#include <cstdint>
#include "edyn/shapes/paged_mesh_shape.hpp"

namespace edyn {

template<typename Archive>
void serialize(Archive &archive, paged_mesh_shape &s) {
    if constexpr(Archive::is_output::value) {

    } else {

    }
}

}

#endif // EDYN_SERIALIZATION_SHAPE_PAGED_MESH_SHAPE_S11N_HPP