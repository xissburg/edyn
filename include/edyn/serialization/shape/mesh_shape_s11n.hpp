#ifndef EDYN_SERIALIZATION_SHAPE_MESH_SHAPE_S11N_HPP
#define EDYN_SERIALIZATION_SHAPE_MESH_SHAPE_S11N_HPP

#include <cstdint>
#include "edyn/shapes/mesh_shape.hpp"

namespace edyn {

template<typename Archive>
void serialize(Archive &archive, mesh_shape &shape) {
    archive(shape.trimesh);
}

}

#endif // EDYN_SERIALIZATION_SHAPE_MESH_SHAPE_S11N_HPP