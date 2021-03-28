#include "edyn/sys/update_rotated_convex_meshes.hpp"
#include "edyn/comp/shape.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/rotated_convex_mesh.hpp"
#include "edyn/config/config.h"
#include "edyn/math/quaternion.hpp"
#include "edyn/shapes/polyhedron_shape.hpp"
#include <entt/entt.hpp>
#include <variant>

namespace edyn {

void update_rotated_convex_meshes(entt::registry &registry) {
    auto view = registry.view<orientation, shape, rotated_convex_mesh>();
    view.each([] (orientation &orn, shape &sh, rotated_convex_mesh &rotated_mesh) {
        EDYN_ASSERT(std::holds_alternative<polyhedron_shape>(sh.var));
        auto &polyhedron = std::get<polyhedron_shape>(sh.var);
        EDYN_ASSERT(polyhedron.mesh->vertices.size() == rotated_mesh.vertices.size());
        EDYN_ASSERT(polyhedron.mesh->normals.size() == rotated_mesh.normals.size());

        for (size_t i = 0; i < polyhedron.mesh->vertices.size(); ++i) {
            auto &vertex_local = polyhedron.mesh->vertices[i];
            rotated_mesh.vertices[i] = rotate(orn, vertex_local);
        }

        for (size_t i = 0; i < polyhedron.mesh->normals.size(); ++i) {
            auto &normal_local = polyhedron.mesh->normals[i];
            rotated_mesh.normals[i] = rotate(orn, normal_local);
        }
    });
}

}