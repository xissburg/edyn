#include "edyn/sys/update_polyhedrons.hpp"
#include "edyn/comp/shape.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/config/config.h"
#include "edyn/shapes/polyhedron_shape.hpp"
#include "edyn/util/aabb_util.hpp"
#include <entt/entt.hpp>
#include <variant>

namespace edyn {

static void update_rotated_mesh_vertices(rotated_mesh &rotated, const convex_mesh &mesh, 
                                         const quaternion &orn) {
    EDYN_ASSERT(mesh.vertices.size() == rotated.vertices.size());

    for (size_t i = 0; i < mesh.vertices.size(); ++i) {
        auto &vertex_local = mesh.vertices[i];
        rotated.vertices[i] = rotate(orn, vertex_local);
    }
}

static void update_rotated_mesh_vertices_and_aabb(rotated_mesh &rotated, AABB &aabb, 
                                                  const convex_mesh &mesh, 
                                                  const vector3 &pos, 
                                                  const quaternion &orn) {
    EDYN_ASSERT(mesh.vertices.size() == rotated.vertices.size());
    EDYN_ASSERT(!mesh.vertices.empty());

    aabb.min = vector3_max;
    aabb.max = -vector3_max;

    for (size_t i = 0; i < mesh.vertices.size(); ++i) {
        auto &vertex_local = mesh.vertices[i];
        auto vertex_rot = rotate(orn, vertex_local);
        rotated.vertices[i] = vertex_rot;

        aabb.min = min(aabb.min, vertex_rot);
        aabb.max = max(aabb.max, vertex_rot);
    }

    aabb.min += pos;
    aabb.max += pos;
}

static void update_rotated_mesh_normals(rotated_mesh &rotated, const convex_mesh &mesh, 
                                        const quaternion &orn) {
    EDYN_ASSERT(mesh.normals.size() == rotated.normals.size());

    for (size_t i = 0; i < mesh.normals.size(); ++i) {
        auto &normal_local = mesh.normals[i];
        rotated.normals[i] = rotate(orn, normal_local);
    }
}

void update_rotated_mesh(rotated_mesh &rotated, const convex_mesh &mesh, 
                         const quaternion &orn) {
    update_rotated_mesh_vertices(rotated, mesh, orn);
    update_rotated_mesh_normals(rotated, mesh, orn);
}

void update_polyhedron(polyhedron_shape &polyhedron, const quaternion &orn) {
    update_rotated_mesh(*polyhedron.rotated, *polyhedron.mesh, orn);
}

void update_polyhedrons(entt::registry &registry) {
    auto poly_view = registry.view<position, orientation, polyhedron_shape, AABB>();
    poly_view.each([] (position &pos, orientation &orn, polyhedron_shape &polyhedron, AABB &aabb) {
        auto &mesh = *polyhedron.mesh;
        auto &rotated = *polyhedron.rotated;
        update_rotated_mesh_vertices_and_aabb(rotated, aabb, mesh, pos, orn);
        update_rotated_mesh_normals(rotated, mesh, orn);
    });

    auto comp_view = registry.view<position, orientation, compound_shape>();
    comp_view.each([] (position &pos, orientation &orn, compound_shape &compound) {
        for (auto &node : compound.nodes) {
            if (!std::holds_alternative<polyhedron_shape>(node.var)) continue;

            auto &polyhedron = std::get<polyhedron_shape>(node.var);
            auto local_orn = orn * node.orientation;
            update_rotated_mesh(*polyhedron.rotated, *polyhedron.mesh, local_orn);
        }
    });
}

}
