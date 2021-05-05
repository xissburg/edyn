#include "edyn/sys/update_rotated_meshes.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/rotated_mesh_list.hpp"
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

void update_rotated_meshes(entt::registry &registry) {
    auto rotated_view = registry.view<rotated_mesh_list>();
    auto view = registry.view<orientation, rotated_mesh_list>();
    view.each([&rotated_view] (orientation &orn, rotated_mesh_list &rotated_list) {
        auto *rot_list_ptr = &rotated_list;

        while(true) {
            // TODO: `rot_list_ptr->orientation` is often `quaternion_identity`.
            // What could be done to avoid this often unnecessary multiplication?
            update_rotated_mesh(*rot_list_ptr->rotated, *rot_list_ptr->mesh, orn * rot_list_ptr->orientation);

            if (rot_list_ptr->next == entt::null) {
                break;
            }

            rot_list_ptr = &rotated_view.get(rot_list_ptr->next);
        }
    });
}

}
