#ifndef EDYN_COLLISION_RAYCAST_HPP
#define EDYN_COLLISION_RAYCAST_HPP

#include <variant>
#include <entt/fwd.hpp>
#include <entt/entity/entity.hpp>
#include "edyn/math/vector3.hpp"
#include "edyn/shapes/cylinder_shape.hpp"
#include "edyn/shapes/capsule_shape.hpp"

namespace edyn {

struct box_shape;
struct sphere_shape;
struct polyhedron_shape;
struct compound_shape;
struct plane_shape;
struct mesh_shape;
struct paged_mesh_shape;

struct box_raycast_info {
    size_t face_index;
};

struct cylinder_raycast_info {
    cylinder_feature feature;
    size_t face_index;
};

struct capsule_raycast_info {
    capsule_feature feature;
    size_t hemisphere_index;
};

struct polyhedron_raycast_info {
    size_t face_index;
};

struct mesh_raycast_info {
    size_t triangle_index;
};

struct paged_mesh_raycast_info {
    size_t submesh_index;
    size_t triangle_index;
};

struct compound_raycast_info {
    size_t child_index;
    std::variant<
        box_raycast_info,
        cylinder_raycast_info,
        capsule_raycast_info,
        polyhedron_raycast_info
    > child_info_var;
};

struct shape_raycast_result {
    scalar proportion { EDYN_SCALAR_MAX };
    vector3 normal;
    std::variant<
        box_raycast_info,
        cylinder_raycast_info,
        capsule_raycast_info,
        polyhedron_raycast_info,
        compound_raycast_info,
        mesh_raycast_info,
        paged_mesh_raycast_info
    > info_var;
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
