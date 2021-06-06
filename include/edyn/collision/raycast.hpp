#ifndef EDYN_COLLISION_RAYCAST_HPP
#define EDYN_COLLISION_RAYCAST_HPP

#include <entt/fwd.hpp>
#include "edyn/math/scalar.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/shapes/shapes.hpp"

namespace edyn {

struct raycast_result {
    entt::entity entity { entt::null };
    scalar proportion { EDYN_SCALAR_MAX };
};

raycast_result raycast(entt::registry &registry, vector3 p0, vector3 p1);

struct raycast_context {
    vector3 pos;
    quaternion orn;
    vector3 p0;
    vector3 p1;
};

scalar raycast(const box_shape &, const raycast_context &);
scalar raycast(const cylinder_shape &, const raycast_context &);
scalar raycast(const sphere_shape &, const raycast_context &);
scalar raycast(const capsule_shape &, const raycast_context &);
scalar raycast(const polyhedron_shape &, const raycast_context &);
scalar raycast(const compound_shape &, const raycast_context &);
scalar raycast(const plane_shape &, const raycast_context &);
scalar raycast(const mesh_shape &, const raycast_context &);
scalar raycast(const paged_mesh_shape &, const raycast_context &);

}

#endif // EDYN_COLLISION_RAYCAST_HPP
