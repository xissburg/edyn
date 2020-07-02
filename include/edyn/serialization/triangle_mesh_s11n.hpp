#ifndef EDYN_SERIALIZATION_TRIANGLE_MESH_S11N_HPP
#define EDYN_SERIALIZATION_TRIANGLE_MESH_S11N_HPP

#include "edyn/shapes/triangle_mesh.hpp"

namespace edyn {

template<typename Archive>
void serialize(Archive &archive, triangle_mesh &tri_mesh) {
    archive(tri_mesh.vertices);
    archive(tri_mesh.indices);
    archive(tri_mesh.cos_angles);
    archive(tri_mesh.is_concave_edge);
    archive(tri_mesh.tree);
}

inline
size_t serialization_sizeof(const triangle_mesh &tri_mesh) {
    return 
        serialization_sizeof(tri_mesh.vertices) +
        serialization_sizeof(tri_mesh.indices) +
        serialization_sizeof(tri_mesh.cos_angles) +
        serialization_sizeof(tri_mesh.is_concave_edge) +
        serialization_sizeof(tri_mesh.tree);
}

}

#endif // EDYN_SERIALIZATION_TRIANGLE_MESH_S11N_HPP