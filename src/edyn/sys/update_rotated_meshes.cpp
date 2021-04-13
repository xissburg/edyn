#include "edyn/sys/update_rotated_meshes.hpp"
#include "edyn/comp/shape.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/rotated_mesh.hpp"
#include "edyn/config/config.h"
#include "edyn/shapes/convex_mesh.hpp"
#include "edyn/shapes/polyhedron_shape.hpp"
#include <entt/entt.hpp>
#include <variant>

namespace edyn {

static void update_rotated_mesh_vertices(rotated_mesh &rmesh, const convex_mesh &mesh, 
                                         const quaternion &orn) {
    EDYN_ASSERT(mesh.vertices.size() == rmesh.vertices.size());

    for (size_t i = 0; i < mesh.vertices.size(); ++i) {
        auto &vertex_local = mesh.vertices[i];
        rmesh.vertices[i] = rotate(orn, vertex_local);
    }
}

static void update_rotated_mesh_vertices_and_aabb(rotated_mesh &rmesh, AABB &aabb, 
                                                  const convex_mesh &mesh, 
                                                  const vector3 &pos, 
                                                  const quaternion &orn) {
    EDYN_ASSERT(mesh.vertices.size() == rmesh.vertices.size());

    aabb.min = vector3_max;
    aabb.max = -vector3_max;

    for (size_t i = 0; i < mesh.vertices.size(); ++i) {
        auto &vertex_local = mesh.vertices[i];
        auto vertex_rot = rotate(orn, vertex_local);
        rmesh.vertices[i] = vertex_rot;

        auto vertex_world = vertex_rot + pos;
        aabb.min = min(aabb.min, vertex_world);
        aabb.max = max(aabb.max, vertex_world);
    }
}

static void update_rotated_mesh_normals(rotated_mesh &rmesh, const convex_mesh &mesh, 
                                        const quaternion &orn) {
    EDYN_ASSERT(mesh.normals.size() == rmesh.normals.size());

    for (size_t i = 0; i < mesh.normals.size(); ++i) {
        auto &normal_local = mesh.normals[i];
        rmesh.normals[i] = rotate(orn, normal_local);
    }
}

void update_rotated_mesh(rotated_mesh &rmesh, const convex_mesh &mesh, const quaternion &orn) {
    update_rotated_mesh_vertices(rmesh, mesh, orn);
    update_rotated_mesh_normals(rmesh, mesh, orn);
}

void update_rotated_meshes(entt::registry &registry) {
    auto view = registry.view<position, orientation, shape, rotated_mesh, AABB>();
    view.each([] (position &pos, orientation &orn, shape &sh, rotated_mesh &rmesh, AABB &aabb) {
        EDYN_ASSERT(std::holds_alternative<polyhedron_shape>(sh.var));
        auto &polyhedron = std::get<polyhedron_shape>(sh.var);
        auto &mesh = *polyhedron.mesh;
        update_rotated_mesh_vertices_and_aabb(rmesh, aabb, mesh, pos, orn);
        update_rotated_mesh_normals(rmesh, mesh, orn);
    });
}

}
