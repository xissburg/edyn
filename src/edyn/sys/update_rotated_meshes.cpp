#include "edyn/sys/update_rotated_meshes.hpp"
#include "edyn/comp/shape.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/rotated_mesh.hpp"
#include "edyn/config/config.h"
#include "edyn/math/quaternion.hpp"
#include "edyn/shapes/convex_mesh.hpp"
#include "edyn/shapes/polyhedron_shape.hpp"
#include <entt/entt.hpp>
#include <variant>

namespace edyn {

void update_rotated_meshes(entt::registry &registry) {
    auto view = registry.view<orientation, shape, rotated_mesh>();
    view.each([] (orientation &orn, shape &sh, rotated_mesh &rmesh) {
        EDYN_ASSERT(std::holds_alternative<polyhedron_shape>(sh.var));
        auto &polyhedron = std::get<polyhedron_shape>(sh.var);
        update_rotated_mesh(rmesh, *polyhedron.mesh, orn);
    });
}

void update_rotated_mesh(rotated_mesh &rmesh, const convex_mesh &mesh, const quaternion &orn) {
    EDYN_ASSERT(mesh.vertices.size() == rmesh.vertices.size());
    EDYN_ASSERT(mesh.normals.size() == rmesh.normals.size());

    for (size_t i = 0; i < mesh.vertices.size(); ++i) {
        auto &vertex_local = mesh.vertices[i];
        rmesh.vertices[i] = rotate(orn, vertex_local);
    }

    for (size_t i = 0; i < mesh.normals.size(); ++i) {
        auto &normal_local = mesh.normals[i];
        rmesh.normals[i] = rotate(orn, normal_local);
    }
}

}
