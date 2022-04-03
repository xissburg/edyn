#include "edyn/sys/update_rotated_meshes.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/rotated_mesh_list.hpp"
#include <entt/entity/registry.hpp>
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
    EDYN_ASSERT(mesh.relevant_normals.size() == rotated.relevant_normals.size());

    for (size_t i = 0; i < mesh.relevant_normals.size(); ++i) {
        auto &normal_local = mesh.relevant_normals[i];
        rotated.relevant_normals[i] = rotate(orn, normal_local);
    }
}

static void update_rotated_mesh_edges(rotated_mesh &rotated, const convex_mesh &mesh,
                                      const quaternion &orn) {
    EDYN_ASSERT(mesh.relevant_edges.size() == rotated.relevant_edges.size());

    for (size_t i = 0; i < mesh.relevant_edges.size(); ++i) {
        auto &edge_local = mesh.relevant_edges[i];
        rotated.relevant_edges[i] = rotate(orn, edge_local);
    }
}

void update_rotated_mesh(rotated_mesh &rotated, const convex_mesh &mesh,
                         const quaternion &orn) {
    update_rotated_mesh_vertices(rotated, mesh, orn);
    update_rotated_mesh_normals(rotated, mesh, orn);
    update_rotated_mesh_edges(rotated, mesh, orn);
}

template<typename RotatedView, typename OrientationView>
void update_rotated_mesh(entt::entity entity, RotatedView &rotated_view, OrientationView &orn_view) {
    auto &orn = orn_view.template get<orientation>(entity);
    auto &rotated_list = rotated_view.template get<rotated_mesh_list>(entity);

    auto *rot_list_ptr = &rotated_list;

    while (true) {
        // TODO: `rot_list_ptr->orientation` is often `quaternion_identity`.
        // What could be done to avoid this often unnecessary multiplication?
        update_rotated_mesh(*rot_list_ptr->rotated, *rot_list_ptr->mesh, orn * rot_list_ptr->orientation);

        if (rot_list_ptr->next == entt::null) {
            break;
        }

        rot_list_ptr = &rotated_view.template get<rotated_mesh_list>(rot_list_ptr->next);
    }
}

void update_rotated_mesh(entt::registry &registry, entt::entity entity) {
    auto rotated_view = registry.view<rotated_mesh_list>();
    auto orn_view = registry.view<orientation>();
    update_rotated_mesh(entity, rotated_view, orn_view);
}

void update_rotated_meshes(entt::registry &registry) {
    auto rotated_view = registry.view<rotated_mesh_list>();
    auto view = registry.view<orientation, rotated_mesh_list>();

    for (auto entity : view) {
        update_rotated_mesh(entity, rotated_view, view);
    }
}

}
