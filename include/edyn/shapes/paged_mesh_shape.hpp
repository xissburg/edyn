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

}

#endif // EDYN_SHAPES_PAGED_MESH_SHAPE_HPP