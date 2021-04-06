#ifndef EDYN_COLLISION_COLLIDE_HPP
#define EDYN_COLLISION_COLLIDE_HPP

#include <optional>
#include <functional>
#include "edyn/comp/shape.hpp"
#include "edyn/comp/rotated_mesh.hpp"

namespace edyn {

struct collision_context {
    vector3 posA;
    quaternion ornA;
    vector3 posB;
    quaternion ornB;
    scalar threshold;

    using rotated_mesh_opt_ref = std::optional<std::reference_wrapper<rotated_mesh>>;
    rotated_mesh_opt_ref rmeshA;
    rotated_mesh_opt_ref rmeshB;

    collision_context swapped() const {
        return {posB, ornB,
                posA, ornA,
                threshold,
                rmeshB, rmeshA};
    }
};

// Calls `collide` with the `shA` and `shB` parameters swapped and swaps
// the returned result so the contact pivots and normals match up with the
// order of shapes A and B.
template<typename ShapeAType, typename ShapeBType>
collision_result swap_collide(const ShapeAType &shA, const ShapeBType &shB,
                              const collision_context &ctx);

// Sphere-Sphere
collision_result collide(const sphere_shape &shA, const sphere_shape &shB,
                         const collision_context &ctx);

// Plane-Plane
inline
collision_result collide(const plane_shape &shA, const plane_shape &shB,
                         const collision_context &ctx) {
    return {}; // collision between infinite planes is undefined here.
}

// Sphere-Plane
collision_result collide(const sphere_shape &shA, const plane_shape &shB,
                         const collision_context &ctx);

// Plane-Sphere
collision_result collide(const plane_shape &shA, const sphere_shape &shB,
                         const collision_context &ctx);

// Cylinder-Cylinder
collision_result collide(const cylinder_shape &shA, const cylinder_shape &shB,
                         const collision_context &ctx);

// Cylinder-Plane
collision_result collide(const cylinder_shape &shA, const plane_shape &shB,
                         const collision_context &ctx);

// Plane-Cylinder
collision_result collide(const plane_shape &shA, const cylinder_shape &shB,
                         const collision_context &ctx);

// Cylinder-Sphere
collision_result collide(const cylinder_shape &shA, const sphere_shape &shB,
                         const collision_context &ctx);

// Sphere-Cylinder
collision_result collide(const sphere_shape &shA, const cylinder_shape &shB,
                         const collision_context &ctx);

// Capsule-Capsule
collision_result collide(const capsule_shape &shA, const capsule_shape &shB,
                         const collision_context &ctx);

// Capsule-Plane
collision_result collide(const capsule_shape &shA, const plane_shape &shB,
                         const collision_context &ctx);

// Plane-Capsule
collision_result collide(const plane_shape &shA, const capsule_shape &shB,
                         const collision_context &ctx);

// Capsule-Sphere
collision_result collide(const capsule_shape &shA, const sphere_shape &shB, 
                         const collision_context &ctx);

// Sphere-Capsule
collision_result collide(const sphere_shape &shA, const capsule_shape &shB,
                         const collision_context &ctx);

// Capsule-Cylinder
collision_result collide(const capsule_shape &shA, const cylinder_shape &shB,
                         const collision_context &ctx);

// Cylinder-Capsule
collision_result collide(const cylinder_shape &shA, const capsule_shape &shB,
                         const collision_context &ctx);

// Mesh-Mesh
inline
collision_result collide(const mesh_shape &shA, const mesh_shape &shB,
                         const collision_context &ctx) {
    return {}; // collision between triangle meshes still undefined.
}

// Plane-Mesh
inline
collision_result collide(const plane_shape &shA, const mesh_shape &shB,
                         const collision_context &ctx) {
    return {}; // collision between triangle meshes and planes still undefined.
}

// Mesh-Plane
inline
collision_result collide(const mesh_shape &shA, const plane_shape &shB,
                         const collision_context &ctx) {
    return swap_collide(shA, shB, ctx);
}

// Sphere-Mesh
collision_result collide(const sphere_shape &shA, const mesh_shape &shB,
                         const collision_context &ctx);

// Mesh-Sphere
collision_result collide(const mesh_shape &shA, const sphere_shape &shB,
                         const collision_context &ctx);

// Capsule-Mesh
collision_result collide(const capsule_shape &shA, const mesh_shape &shB,
                         const collision_context &ctx);

// Mesh-Capsule
collision_result collide(const mesh_shape &shA, const capsule_shape &shB,
                         const collision_context &ctx);

// Cylinder-Mesh
collision_result collide(const cylinder_shape &shA, const mesh_shape &shB,
                         const collision_context &ctx);

// Mesh-Cylinder
collision_result collide(const mesh_shape &shA, const cylinder_shape &shB,
                         const collision_context &ctx);

// Box-Box
collision_result collide(const box_shape &shA, const box_shape &shB,
                         const collision_context &ctx);

// Box-Plane
collision_result collide(const box_shape &shA, const plane_shape &shB,
                         const collision_context &ctx);

// Plane-Box
collision_result collide(const plane_shape &shA, const box_shape &shB,
                         const collision_context &ctx);

// Sphere-Box
collision_result collide(const sphere_shape &shA, const box_shape &shB,
                         const collision_context &ctx);

// Box-Sphere
collision_result collide(const box_shape &shA, const sphere_shape &shB,
                         const collision_context &ctx);

// Capsule-Box
collision_result collide(const capsule_shape &shA, const box_shape &shB,
                         const collision_context &ctx);

// Box-Capsule
collision_result collide(const box_shape &shA, const capsule_shape &shB,
                         const collision_context &ctx);

// Cylinder-Box
collision_result collide(const cylinder_shape &shA, const box_shape &shB,
                         const collision_context &ctx);

// Box-Cylinder
collision_result collide(const box_shape &shA, const cylinder_shape &shB,
                         const collision_context &ctx);

// Box-Mesh
collision_result collide(const box_shape &shA, const mesh_shape &shB,
                         const collision_context &ctx);

// Mesh-Box
collision_result collide(const mesh_shape &shA, const box_shape &shB,
                         const collision_context &ctx);

// Paged Mesh-Paged Mesh
inline
collision_result collide(const paged_mesh_shape &shA, const paged_mesh_shape &shB,
                         const collision_context &ctx) {
    return {}; // collision between paged triangle meshes still undefined.
}

// Plane-Paged Mesh
inline
collision_result collide(const plane_shape &shA, const paged_mesh_shape &shB,
                         const collision_context &ctx) {
    return {}; // collision between paged triangle meshes and planes still undefined.
}

// Paged Mesh-Plane
inline
collision_result collide(const paged_mesh_shape &shA, const plane_shape &shB,
                         const collision_context &ctx) {
    return swap_collide(shA, shB, ctx);
}

// Sphere-Paged Mesh
collision_result collide(const sphere_shape &shA, const paged_mesh_shape &shB,
                         const collision_context &ctx);

// Paged Mesh-Sphere
collision_result collide(const paged_mesh_shape &shA, const sphere_shape &shB,
                         const collision_context &ctx);

// Capsule-Paged Mesh
collision_result collide(const capsule_shape &shA, const paged_mesh_shape &shB,
                         const collision_context &ctx);

// Paged Mesh-Capsule
collision_result collide(const paged_mesh_shape &shA, const capsule_shape &shB,
                         const collision_context &ctx);

// Cylinder-Paged Mesh
collision_result collide(const cylinder_shape &shA, const paged_mesh_shape &shB,
                         const collision_context &ctx);

// Paged Mesh-Cylinder
collision_result collide(const paged_mesh_shape &shA, const cylinder_shape &shB,
                         const collision_context &ctx);

// Box-Paged Mesh
collision_result collide(const box_shape &shA, const paged_mesh_shape &shB,
                         const collision_context &ctx);

// Paged Mesh-Box
collision_result collide(const paged_mesh_shape &shA, const box_shape &shB,
                         const collision_context &ctx);

// Mesh-Paged Mesh
inline
collision_result collide(const mesh_shape &shA, const paged_mesh_shape &shB,
                         const collision_context &ctx) {
    return {}; // collision between triangle meshes still undefined.
}

// Paged Mesh-Mesh
inline
collision_result collide(const paged_mesh_shape &shA, const mesh_shape &shB,
                         const collision_context &ctx) {
    return swap_collide(shA, shB, ctx);
}

// Sphere-Triangle
void collide_sphere_triangle(
    const sphere_shape &, const vector3 &sphere_pos, const quaternion &sphere_orn,
    const triangle_shape &tri, scalar threshold, collision_result &result);

// Cylinder-Triangle
void collide_cylinder_triangle(
    const cylinder_shape &, const vector3 &posA, const quaternion &ornA,
    const vector3 &disc_center_pos, const vector3 &disc_center_neg,
    const vector3 &cylinder_axis, const triangle_shape &tri, 
    scalar threshold, collision_result &result);

// Box-Triangle
void collide_box_triangle(
    const box_shape &, const vector3 &box_pos, const quaternion &box_orn,
    const std::array<vector3, 3> box_axes, const triangle_shape &tri,
    scalar threshold, collision_result &result);

// Polyhedron-Polyhedron
collision_result collide(const polyhedron_shape &shA, const polyhedron_shape &shB,
                         const collision_context &ctx);

// Polyhedron-Plane
collision_result collide(const polyhedron_shape &shA, const plane_shape &shB,
                         const collision_context &ctx);

// Plane-Polyhedron
collision_result collide(const plane_shape &shA, const polyhedron_shape &shB,
                         const collision_context &ctx);

// Sphere-Polyhedron
collision_result collide(const sphere_shape &shA, const polyhedron_shape &shB,
                         const collision_context &ctx);

// Polyhedron-Sphere
collision_result collide(const polyhedron_shape &shA, const sphere_shape &shB,
                         const collision_context &ctx);

// Box-Polyhedron
collision_result collide(const box_shape &shA, const polyhedron_shape &shB,
                         const collision_context &ctx);

// Polyhedron-Box
collision_result collide(const polyhedron_shape &shA, const box_shape &shB,
                         const collision_context &ctx);

// Capsule-Polyhedron
collision_result collide(const capsule_shape &shA, const polyhedron_shape &shB,
                         const collision_context &ctx);

// Polyhedron-Capsule
collision_result collide(const polyhedron_shape &shA, const capsule_shape &shB,
                         const collision_context &ctx);

// Cylinder-Polyhedron
collision_result collide(const cylinder_shape &shA, const polyhedron_shape &shB,
                         const collision_context &ctx);

// Polyhedron-Cylinder
collision_result collide(const polyhedron_shape &shA, const cylinder_shape &shB,
                         const collision_context &ctx);

// Polyhedron-Mesh
collision_result collide(const polyhedron_shape &shA, const mesh_shape &shB,
                         const collision_context &ctx);

// Mesh-Polyhedron
collision_result collide(const mesh_shape &shA, const polyhedron_shape &shB,
                         const collision_context &ctx);

// Polyhedron-Paged Mesh
collision_result collide(const polyhedron_shape &shA, const paged_mesh_shape &shB,
                         const collision_context &ctx);

// Paged Mesh-Polyhedron
collision_result collide(const paged_mesh_shape &shA, const polyhedron_shape &shB,
                         const collision_context &ctx);

// Polyhedron-Triangle
void collide_polyhedron_triangle(
    const polyhedron_shape &, const rotated_mesh &,
    const vector3 &pos_poly, const quaternion &orn_poly,
    const triangle_shape &tri, scalar threshold, collision_result &result);

template<typename ShapeAType, typename ShapeBType>
collision_result swap_collide(const ShapeAType &shA, const ShapeBType &shB,
                              const collision_context &ctx) {
    return collide(shB, shA, ctx.swapped()).swap(ctx.ornB, ctx.ornA);
}

}

#endif // EDYN_COLLISION_COLLIDE_HPP
