#ifndef EDYN_SHAPES_PAGED_MESH_SHAPE_HPP
#define EDYN_SHAPES_PAGED_MESH_SHAPE_HPP

#include <memory>

#include "edyn/comp/aabb.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/serialization/file_archive.hpp"
#include "edyn/serialization/memory_archive.hpp"
#include "paged_triangle_mesh.hpp"

namespace edyn {

struct paged_mesh_shape {
    std::shared_ptr<paged_triangle_mesh<memory_input_archive_source>> trimesh;

    AABB aabb(const vector3 &pos, const quaternion &orn) const {
        return {trimesh->m_aabb.min + pos, trimesh->m_aabb.max + pos};
    }

    vector3 inertia(scalar mass) const {
        return vector3_max;
    }
};

}

#endif // EDYN_SHAPES_PAGED_MESH_SHAPE_HPP