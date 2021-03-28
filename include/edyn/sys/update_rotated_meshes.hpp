#ifndef EDYN_SYS_UPDATE_ROTATED_MESHES_HPP
#define EDYN_SYS_UPDATE_ROTATED_MESHES_HPP

#include <entt/fwd.hpp>

namespace edyn {

struct convex_mesh;
struct rotated_mesh;
struct quaternion;

void update_rotated_meshes(entt::registry &);
void update_rotated_mesh(rotated_mesh &rmesh, const convex_mesh &mesh, const quaternion &orn);

}

#endif // EDYN_SYS_UPDATE_ROTATED_MESHES_HPP
