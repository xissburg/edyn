#ifndef EDYN_SERIALIZATION_SHAPE_MESH_SHAPE_S11N_HPP
#define EDYN_SERIALIZATION_SHAPE_MESH_SHAPE_S11N_HPP

#include <cstdint>
#include "edyn/shapes/mesh_shape.hpp"

namespace edyn {

template<typename Archive>
void serialize(Archive &archive, mesh_shape &s) {
    if constexpr(Archive::is_output::value) {
        auto *trimesh_ptr = new std::shared_ptr(s.trimesh);
        auto intptr = reinterpret_cast<intptr_t>(trimesh_ptr);
        archive(intptr);
    } else {
        intptr_t intptr;
        archive(intptr);
        auto *trimesh_ptr = reinterpret_cast<std::shared_ptr<triangle_mesh> *>(intptr);
        s.trimesh = *trimesh_ptr;
        delete trimesh_ptr;
    }
}

}

#endif // EDYN_SERIALIZATION_SHAPE_MESH_SHAPE_S11N_HPP