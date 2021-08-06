#ifndef EDYN_SERIALIZATION_SHAPE_POLYHEDRON_S11N_HPP
#define EDYN_SERIALIZATION_SHAPE_POLYHEDRON_S11N_HPP

#include "edyn/shapes/convex_mesh.hpp"
#include "edyn/shapes/polyhedron_shape.hpp"

namespace edyn {

template<typename Archive>
void serialize(Archive &archive, convex_mesh &mesh) {
    archive(mesh.vertices);
    archive(mesh.indices);
    archive(mesh.faces);

    if constexpr(Archive::is_input::value) {
        mesh.initialize();
    }
}

template<typename Archive>
void serialize(Archive &archive, polyhedron_shape &shape) {
    archive(shape.mesh);
}

}

#endif // EDYN_SERIALIZATION_SHAPE_POLYHEDRON_S11N_HPP
