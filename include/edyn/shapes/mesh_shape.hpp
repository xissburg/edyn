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

    AABB aabb(const vector3 &pos, const quaternion &orn) const {
        return {trimesh->get_aabb().min + pos, trimesh->get_aabb().max + pos};
    }

    matrix3x3 inertia(scalar mass) const {
        return diagonal_matrix(vector3_max);
    }
};

}

#endif // EDYN_SHAPES_MESH_SHAPE_HPP