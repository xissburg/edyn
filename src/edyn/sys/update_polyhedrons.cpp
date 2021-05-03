#include "edyn/sys/update_polyhedrons.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/shapes/polyhedron_shape.hpp"
#include "edyn/shapes/compound_shape.hpp"
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
    auto poly_view = registry.view<position, orientation, polyhedron_shape>();
    poly_view.each([] (position &pos, orientation &orn, polyhedron_shape &polyhedron) {
        update_polyhedron(polyhedron, orn);
    });

    // TODO: Not all compounds hold a polyhedron, so this could be very wasteful.
    auto compound_view = registry.view<position, orientation, compound_shape>();
    compound_view.each([] (position &pos, orientation &orn, compound_shape &compound) {
        for (auto &node : compound.nodes) {
            if (!std::holds_alternative<polyhedron_shape>(node.shape_var)) continue;

            auto &polyhedron = std::get<polyhedron_shape>(node.shape_var);
            auto world_orn = orn * node.orientation;
            update_polyhedron(polyhedron, world_orn);
        }
    });
}

}
