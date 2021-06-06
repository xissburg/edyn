#ifndef EDYN_COLLISION_RAYCAST_HPP
#define EDYN_COLLISION_RAYCAST_HPP

#include <entt/fwd.hpp>
#include <entt/entity/entity.hpp>
#include "edyn/math/vector3.hpp"
#include "edyn/collision/shape_feature.hpp"

namespace edyn {

struct shape_raycast_result {
    scalar proportion { EDYN_SCALAR_MAX };
    shape_feature feature;
    vector3 normal;
};

struct raycast_result : public shape_raycast_result {
    entt::entity entity { entt::null };
};

struct raycast_context {
    vector3 pos;
    quaternion orn;
    vector3 p0;
    vector3 p1;
};

raycast_result raycast(entt::registry &registry, vector3 p0, vector3 p1);

shape_raycast_result raycast(const box_shape &, const raycast_context &);
shape_raycast_result raycast(const cylinder_shape &, const raycast_context &);
shape_raycast_result raycast(const sphere_shape &, const raycast_context &);
shape_raycast_result raycast(const capsule_shape &, const raycast_context &);
shape_raycast_result raycast(const polyhedron_shape &, const raycast_context &);
shape_raycast_result raycast(const compound_shape &, const raycast_context &);
shape_raycast_result raycast(const plane_shape &, const raycast_context &);
shape_raycast_result raycast(const mesh_shape &, const raycast_context &);
shape_raycast_result raycast(const paged_mesh_shape &, const raycast_context &);

}

#endif // EDYN_COLLISION_RAYCAST_HPP
