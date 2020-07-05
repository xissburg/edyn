#ifndef EDYN_COLLISION_COLLIDE_HPP
#define EDYN_COLLISION_COLLIDE_HPP

#include "edyn/comp/shape.hpp"

namespace edyn {

// Sphere-Sphere
collision_result collide(const sphere_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const sphere_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold);

// Plane-Plane
inline
collision_result collide(const plane_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const plane_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold) {
    return {}; // collision between infinite planes is undefined here.
}

// Sphere-Plane
collision_result collide(const sphere_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const plane_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold);

// Plane-Sphere
inline
collision_result collide(const plane_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const sphere_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold) {
    return collide(shB, posB, ornB, shA, posA, ornA, threshold).swap(ornA, ornB);
}

// Cylinder-Cylinder
collision_result collide(const cylinder_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const cylinder_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold);

// Cylinder-Plane
collision_result collide(const cylinder_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const plane_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold);

// Plane-Cylinder
inline
collision_result collide(const plane_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const cylinder_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold) {
    return collide(shB, posB, ornB, shA, posA, ornA, threshold).swap(ornA, ornB);
}

// Cylinder-Sphere
collision_result collide(const cylinder_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const sphere_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold);

// Sphere-Cylinder
inline
collision_result collide(const sphere_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const cylinder_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold) {
    return collide(shB, posB, ornB, shA, posA, ornA, threshold).swap(ornA, ornB);
}

// Capsule-Capsule
collision_result collide(const capsule_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const capsule_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold);

// Capsule-Plane
collision_result collide(const capsule_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const plane_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold);

// Plane-Capsule
inline
collision_result collide(const plane_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const capsule_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold) {
    return collide(shB, posB, ornB, shA, posA, ornA, threshold).swap(ornA, ornB);
}

// Capsule-Sphere
collision_result collide(const capsule_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const sphere_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold);

// Sphere-Capsule
inline
collision_result collide(const sphere_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const capsule_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold) {
    return collide(shB, posB, ornB, shA, posA, ornA, threshold).swap(ornA, ornB);
}

// Capsule-Cylinder
collision_result collide(const capsule_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const cylinder_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold);

// Cylinder-Capsule
inline
collision_result collide(const cylinder_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const capsule_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold) {
    return collide(shB, posB, ornB, shA, posA, ornA, threshold).swap(ornA, ornB);
}

// Mesh-Mesh
inline
collision_result collide(const mesh_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const mesh_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold) {
    return {}; // collision between triangle meshes still undefined.
}

// Plane-Mesh
inline
collision_result collide(const plane_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const mesh_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold) {
    return {}; // collision between triangle meshes and planes still undefined.
}

// Mesh-Plane
inline
collision_result collide(const mesh_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const plane_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold) {
    return collide(shB, posB, ornB, shA, posA, ornA, threshold).swap(ornA, ornB);
}

// Sphere-Mesh
collision_result collide(const sphere_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const mesh_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold);

// Mesh-Sphere
inline
collision_result collide(const mesh_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const sphere_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold) {
    return collide(shB, posB, ornB, shA, posA, ornA, threshold).swap(ornA, ornB);
}

// Capsule-Mesh
collision_result collide(const capsule_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const mesh_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold);

// Mesh-Capsule
inline
collision_result collide(const mesh_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const capsule_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold) {
    return collide(shB, posB, ornB, shA, posA, ornA, threshold).swap(ornA, ornB);
}

// Cylinder-Mesh
collision_result collide(const cylinder_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const mesh_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold);

// Mesh-Cylinder
inline
collision_result collide(const mesh_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const cylinder_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold) {
    return collide(shB, posB, ornB, shA, posA, ornA, threshold).swap(ornA, ornB);
}

// Box-Box
collision_result collide(const box_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const box_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold);

// Box-Plane
collision_result collide(const box_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const plane_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold);

// Plane-Box
inline
collision_result collide(const plane_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const box_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold) {
    return collide(shB, posB, ornB, shA, posA, ornA, threshold).swap(ornA, ornB);
}

// Sphere-Box
collision_result collide(const sphere_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const box_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold);

// Box-Sphere
inline
collision_result collide(const box_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const sphere_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold) {
    return collide(shB, posB, ornB, shA, posA, ornA, threshold).swap(ornA, ornB);
}

// Capsule-Box
collision_result collide(const capsule_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const box_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold);

// Box-Capsule
inline
collision_result collide(const box_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const capsule_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold) {
    return collide(shB, posB, ornB, shA, posA, ornA, threshold).swap(ornA, ornB);
}

// Cylinder-Box
collision_result collide(const cylinder_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const box_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold);

// Box-Cylinder
inline
collision_result collide(const box_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const cylinder_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold) {
    return collide(shB, posB, ornB, shA, posA, ornA, threshold).swap(ornA, ornB);
}

// Box-Mesh
collision_result collide(const box_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const mesh_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold);

// Mesh-Box
inline
collision_result collide(const mesh_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const box_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold) {
    return collide(shB, posB, ornB, shA, posA, ornA, threshold).swap(ornA, ornB);
}

// Paged Mesh-Paged Mesh
inline
collision_result collide(const paged_mesh_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const paged_mesh_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold) {
    return {}; // collision between paged triangle meshes still undefined.
}

// Plane-Paged Mesh
inline
collision_result collide(const plane_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const paged_mesh_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold) {
    return {}; // collision between paged triangle meshes and planes still undefined.
}

// Paged Mesh-Plane
inline
collision_result collide(const paged_mesh_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const plane_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold) {
    return collide(shB, posB, ornB, shA, posA, ornA, threshold).swap(ornA, ornB);
}

// Sphere-Paged Mesh
collision_result collide(const sphere_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const paged_mesh_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold);

// Paged Mesh-Sphere
inline
collision_result collide(const paged_mesh_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const sphere_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold) {
    return collide(shB, posB, ornB, shA, posA, ornA, threshold).swap(ornA, ornB);
}

// Capsule-Paged Mesh
collision_result collide(const capsule_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const paged_mesh_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold);

// Paged Mesh-Capsule
inline
collision_result collide(const paged_mesh_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const capsule_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold) {
    return collide(shB, posB, ornB, shA, posA, ornA, threshold).swap(ornA, ornB);
}

// Cylinder-Paged Mesh
collision_result collide(const cylinder_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const paged_mesh_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold);

// Paged Mesh-Cylinder
inline
collision_result collide(const paged_mesh_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const cylinder_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold) {
    return collide(shB, posB, ornB, shA, posA, ornA, threshold).swap(ornA, ornB);
}

// Box-Paged Mesh
collision_result collide(const box_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const paged_mesh_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold);

// Paged Mesh-Box
inline
collision_result collide(const paged_mesh_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const box_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold) {
    return collide(shB, posB, ornB, shA, posA, ornA, threshold).swap(ornA, ornB);
}

// Mesh-Paged Mesh
inline
collision_result collide(const mesh_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const paged_mesh_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold) {
    return {}; // collision between triangle meshes still undefined.
}

// Paged Mesh-Mesh
inline
collision_result collide(const paged_mesh_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const mesh_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold) {
    return collide(shB, posB, ornB, shA, posA, ornA, threshold).swap(ornA, ornB);
}

// Sphere-Triangle

void collide_sphere_triangle(
    const sphere_shape &sphere, const vector3 &sphere_pos, const quaternion &sphere_orn,
    const triangle_vertices &vertices, const std::array<bool, 3> &is_concave_edge, 
    const std::array<scalar, 3> &cos_angles, scalar threshold, collision_result &result);

// Cylinder-Triangle
void collide_cylinder_triangle(
    const cylinder_shape &, const vector3 &posA, const quaternion &ornA,
    const vector3 &disc_center_pos, const vector3 &disc_center_neg,
    const vector3 &cylinder_axis, const triangle_vertices &, 
    const std::array<bool, 3> &is_concave_edge, const std::array<scalar, 3> &cos_angles, 
    scalar threshold, collision_result &);
}

#endif // EDYN_COLLISION_COLLIDE_HPP