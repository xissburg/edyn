#ifndef EDYN_SHAPES_PAGED_MESH_SHAPE_HPP
#define EDYN_SHAPES_PAGED_MESH_SHAPE_HPP

#include <memory>
#include "paged_triangle_mesh.hpp"

namespace edyn {

/**
 * @brief A concave triangle mesh shape with paging support.
 * @remarks Paged triangle meshes can only be assigned to static rigid bodies.
 * The `collide` functions involving this shape ignore position and
 * orientation. If the mesh needs to be transformed, do so while constructing
 * or loading it.
 */
struct paged_mesh_shape {
    std::shared_ptr<paged_triangle_mesh> trimesh;
};

template<typename Archive>
void serialize(Archive &archive, paged_mesh_shape &s) {
    if constexpr(Archive::is_output::value) {

    } else {

    }
}

}

#endif // EDYN_SHAPES_PAGED_MESH_SHAPE_HPP