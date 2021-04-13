#ifndef EDYN_SHAPES_MESH_SHAPE_HPP
#define EDYN_SHAPES_MESH_SHAPE_HPP

#include <memory>

#include "edyn/comp/aabb.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/math/quaternion.hpp"
#include "triangle_mesh.hpp"

namespace edyn {

struct mesh_shape {
    std::shared_ptr<triangle_mesh> trimesh;
};

}

#endif // EDYN_SHAPES_MESH_SHAPE_HPP