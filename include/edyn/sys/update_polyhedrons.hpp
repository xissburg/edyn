#ifndef EDYN_SYS_UPDATE_POLYHEDRONS_HPP
#define EDYN_SYS_UPDATE_POLYHEDRONS_HPP

#include <entt/fwd.hpp>

namespace edyn {

struct polyhedron_shape;
struct rotated_mesh;
struct convex_mesh;
struct quaternion;

/**
 * @brief Updates the rotated mesh of all polyhedron shapes, including the ones
 * in compound shapes.
 * @param registry Source of polyhedrons.
 */
void update_polyhedrons(entt::registry &registry);

/**
 * @brief Updates the rotated mesh of a single polyhedron.
 * @param polyhedron The polyhedron to be updated.
 * @param orn Orientation to be applied to the polyhedron's convex mesh
 * vertices and normals.
 */
void update_polyhedron(polyhedron_shape &polyhedron, const quaternion &orn);

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

#endif // EDYN_SYS_UPDATE_POLYHEDRONS_HPP
