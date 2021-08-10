#ifndef EDYN_SYS_UPDATE_ROTATED_MESHES_HPP
#define EDYN_SYS_UPDATE_ROTATED_MESHES_HPP

#include <entt/entity/fwd.hpp>

namespace edyn {

struct rotated_mesh;
struct convex_mesh;
struct quaternion;

/**
 * @brief Updates the rotated mesh of all polyhedron shapes, including the ones
 * in compound shapes.
 * @param registry Source of shapes.
 */
void update_rotated_meshes(entt::registry &registry);

/**
 * @brief Updates the rotated mesh of a single entity, which is assumed to have
 * either a polyhedron or a compound shape.
 * @param registry Data source.
 * @param entity Entity to be updated.
 */
void update_rotated_mesh(entt::registry &registry, entt::entity entity);

/**
 * @brief Updates rotated mesh by appliying a rotation to the vertex positions
 * and face normals of a mesh.
 * @param rotated The rotated mesh to be updated.
 * @param mesh The source convex mesh.
 * @param orn Rotation to be applied.
 */
void update_rotated_mesh(rotated_mesh &rotated, const convex_mesh &mesh,
                         const quaternion &orn);

}

#endif // EDYN_SYS_UPDATE_ROTATED_MESHES_HPP
