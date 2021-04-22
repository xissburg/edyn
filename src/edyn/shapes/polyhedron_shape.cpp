#include "edyn/shapes/polyhedron_shape.hpp"
#include "edyn/util/shape_util.hpp"
#include "edyn/util/shape_util.hpp"

namespace edyn {

polyhedron_shape::polyhedron_shape(const std::string &path_to_obj,
                                   const vector3 &pos,
                                   const quaternion &orn,
                                   const vector3 &scale)
    : mesh(std::make_shared<convex_mesh>())
{
    auto meshes = std::vector<obj_mesh>{};
    load_meshes_from_obj(path_to_obj, meshes, pos, orn, scale);
    EDYN_ASSERT(meshes.size() == 1);

    auto &m = meshes.front();
    mesh->vertices = std::move(m.vertices);
    mesh->indices = std::move(m.indices);
    mesh->faces = std::move(m.faces);

    mesh->calculate_normals();
    mesh->calculate_edges();

#ifdef EDYN_DEBUG
    mesh->validate();
#endif
}

}
