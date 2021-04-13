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
    calculate_local_aabb();

#ifdef EDYN_DEBUG
    mesh->validate();
#endif
}

void polyhedron_shape::calculate_local_aabb() {
    local_aabb = point_cloud_aabb(mesh->vertices);
}

AABB polyhedron_shape::aabb(const vector3 &pos, const quaternion &orn) const {
    // For large meshes use the AABB of the transformed local AABB instead of
    // recalculating a tight fitting AABB every time.
    constexpr size_t convex_mesh_exact_aabb_vertex_count_limit = 24;

    if (mesh->vertices.size() < convex_mesh_exact_aabb_vertex_count_limit) {
        return point_cloud_aabb(mesh->vertices, pos, orn);
    } else {
        return aabb_of_aabb(local_aabb, pos, orn);
    }
}

}
