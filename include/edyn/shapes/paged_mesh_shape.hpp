#ifndef EDYN_SHAPES_PAGED_MESH_SHAPE_HPP
#define EDYN_SHAPES_PAGED_MESH_SHAPE_HPP

#include <memory>

#include "edyn/comp/aabb.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/serialization/file_archive.hpp"
#include "edyn/serialization/memory_archive.hpp"
#include "paged_triangle_mesh.hpp"

namespace edyn {

struct paged_mesh_shape {
    std::shared_ptr<paged_triangle_mesh> trimesh;
};

}

#endif // EDYN_SHAPES_PAGED_MESH_SHAPE_HPP