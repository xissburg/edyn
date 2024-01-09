#include "edyn/sys/update_rotated_meshes.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/rotated_mesh_list.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/util/island_util.hpp"
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

template<typename RotatedView, typename OrientationView>
void update_rotated_mesh(entt::entity entity, RotatedView &rotated_view, OrientationView &orn_view) {
    const auto &orn = orn_view.template get<orientation>(entity);

    do {
        auto &rotated = rotated_view.template get<rotated_mesh_list>(entity);
        // TODO: `rot_list_ptr->orientation` is often `quaternion_identity`.
        // What could be done to avoid this often unnecessary multiplication?
        update_rotated_mesh(*rotated.rotated, *rotated.mesh, orn * rotated.orientation);
        entity = rotated.next;
    } while (entity != entt::null);
}

void update_rotated_mesh(entt::registry &registry, entt::entity entity) {
    auto rotated_view = registry.view<rotated_mesh_list>();
    auto orn_view = registry.view<orientation>();
    update_rotated_mesh(entity, rotated_view, orn_view);
}

void update_rotated_meshes(entt::registry &registry) {
    auto rotated_view = registry.view<rotated_mesh_list>();
    auto orn_view = registry.view<orientation>();

    for (auto entity : registry.view<rotated_mesh_list, dynamic_tag>(exclude_sleeping_disabled)) {
        update_rotated_mesh(entity, rotated_view, orn_view);
    }

    // TODO: update only kinematic entities that have rotated.
    for (auto entity : registry.view<rotated_mesh_list, kinematic_tag>()) {
        update_rotated_mesh(entity, rotated_view, orn_view);
    }
}

}
