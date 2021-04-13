#include "edyn/shapes/polyhedron_shape.hpp"
#include "edyn/util/shape_util.hpp"
#include "edyn/util/shape_util.hpp"

namespace edyn {

polyhedron_shape::polyhedron_shape(const std::string &path_to_obj)
    : mesh(std::make_shared<convex_mesh>())
{
    load_mesh_from_obj(path_to_obj, mesh->vertices, mesh->indices, mesh->faces);
    mesh->calculate_normals();
    mesh->calculate_edges();

#ifdef EDYN_DEBUG
    mesh->validate();
#endif
}

}
