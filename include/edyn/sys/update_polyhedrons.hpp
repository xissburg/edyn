#ifndef EDYN_SYS_UPDATE_POLYHEDRONS_HPP
#define EDYN_SYS_UPDATE_POLYHEDRONS_HPP

#include <entt/fwd.hpp>

namespace edyn {

struct polyhedron_shape;
struct rotated_mesh;
struct convex_mesh;
struct quaternion;

void update_polyhedrons(entt::registry &);
void update_polyhedron(polyhedron_shape &polyhedron, const quaternion &orn);
void update_rotated_mesh(rotated_mesh &rotated, const convex_mesh &mesh, 
                         const quaternion &orn);

}

#endif // EDYN_SYS_UPDATE_POLYHEDRONS_HPP
