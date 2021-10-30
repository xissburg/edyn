#ifndef EDYN_COLLISION_COLLIDE_HPP
#define EDYN_COLLISION_COLLIDE_HPP

#include "edyn/shapes/shapes.hpp"
#include "edyn/collision/collision_result.hpp"
#include "edyn/util/aabb_util.hpp"
#include "edyn/util/tuple_util.hpp"

namespace edyn {

struct collision_context {
    vector3 posA;
    quaternion ornA;
    AABB aabbA;

    vector3 posB;
    quaternion ornB;
    AABB aabbB;

    scalar threshold;

    collision_context swapped() const {
        return {posB, ornB, aabbB,
                posA, ornA, aabbA,
                threshold};
    }
};

#if defined(_MSC_VER)
// Disable "unreferenced formal parameter" warning.
__pragma(warning(push))
__pragma(warning(disable:4100))
#endif

// Calls `collide` with the `shA` and `shB` parameters swapped and swaps
// the returned result so the contact pivots and normals match up with the
// order of shapes A and B.
template<typename ShapeAType, typename ShapeBType>
void swap_collide(const ShapeAType &shA, const ShapeBType &shB,
                  const collision_context &ctx, collision_result &result);

// Sphere-Triangle Mesh
void collide(const sphere_shape &sphere, const triangle_mesh &mesh,
             const collision_context &ctx, collision_result &result);

// Cylinder-Triangle Mesh
void collide(const cylinder_shape &cylinder, const triangle_mesh &mesh,
             const collision_context &ctx, collision_result &result);

// Capsule-Triangle Mesh
void collide(const capsule_shape &capsule, const triangle_mesh &mesh,
             const collision_context &ctx, collision_result &result);

// Box-Triangle Mesh
void collide(const box_shape &box, const triangle_mesh &mesh,
             const collision_context &ctx, collision_result &result);

// Polyhedron-Triangle Mesh
void collide(const polyhedron_shape &poly, const triangle_mesh &mesh,
             const collision_context &ctx, collision_result &result);

// Compound-Triangle Mesh
void collide(const compound_shape &compound, const triangle_mesh &mesh,
             const collision_context &ctx, collision_result &result);

// Sphere-Sphere
void collide(const sphere_shape &shA, const sphere_shape &shB,
             const collision_context &ctx, collision_result &result);

// Plane-Plane
inline
void collide(const plane_shape &shA, const plane_shape &shB,
             const collision_context &ctx, collision_result &result) {
    // collision between infinite planes is undefined here.
}

// Sphere-Plane
void collide(const sphere_shape &shA, const plane_shape &shB,
             const collision_context &ctx, collision_result &result);

// Plane-Sphere
void collide(const plane_shape &shA, const sphere_shape &shB,
             const collision_context &ctx, collision_result &result);

// Cylinder-Cylinder
void collide(const cylinder_shape &shA, const cylinder_shape &shB,
             const collision_context &ctx, collision_result &result);

// Cylinder-Plane
void collide(const cylinder_shape &shA, const plane_shape &shB,
             const collision_context &ctx, collision_result &result);

// Plane-Cylinder
void collide(const plane_shape &shA, const cylinder_shape &shB,
             const collision_context &ctx, collision_result &result);

// Cylinder-Sphere
void collide(const cylinder_shape &shA, const sphere_shape &shB,
             const collision_context &ctx, collision_result &result);

// Sphere-Cylinder
void collide(const sphere_shape &shA, const cylinder_shape &shB,
             const collision_context &ctx, collision_result &result);

// Capsule-Capsule
void collide(const capsule_shape &shA, const capsule_shape &shB,
             const collision_context &ctx, collision_result &result);

// Capsule-Plane
void collide(const capsule_shape &shA, const plane_shape &shB,
             const collision_context &ctx, collision_result &result);

// Plane-Capsule
void collide(const plane_shape &shA, const capsule_shape &shB,
             const collision_context &ctx, collision_result &result);

// Capsule-Sphere
void collide(const capsule_shape &shA, const sphere_shape &shB,
             const collision_context &ctx, collision_result &result);

// Sphere-Capsule
void collide(const sphere_shape &shA, const capsule_shape &shB,
             const collision_context &ctx, collision_result &result);

// Capsule-Cylinder
void collide(const capsule_shape &shA, const cylinder_shape &shB,
             const collision_context &ctx, collision_result &result);

// Cylinder-Capsule
void collide(const cylinder_shape &shA, const capsule_shape &shB,
             const collision_context &ctx, collision_result &result);

// Mesh-Mesh
inline
void collide(const mesh_shape &shA, const mesh_shape &shB,
             const collision_context &ctx, collision_result &result) {
    // collision between triangle meshes still undefined.
}

// Plane-Mesh
inline
void collide(const plane_shape &shA, const mesh_shape &shB,
             const collision_context &ctx, collision_result &result) {
    // collision between triangle meshes and planes still undefined.
}

// Mesh-Plane
inline
void collide(const mesh_shape &shA, const plane_shape &shB,
             const collision_context &ctx, collision_result &result) {
    swap_collide(shA, shB, ctx, result);
}

// Box-Box
void collide(const box_shape &shA, const box_shape &shB,
             const collision_context &ctx, collision_result &result);

// Box-Plane
void collide(const box_shape &shA, const plane_shape &shB,
             const collision_context &ctx, collision_result &result);

// Plane-Box
void collide(const plane_shape &shA, const box_shape &shB,
             const collision_context &ctx, collision_result &result);

// Sphere-Box
void collide(const sphere_shape &shA, const box_shape &shB,
             const collision_context &ctx, collision_result &result);

// Box-Sphere
void collide(const box_shape &shA, const sphere_shape &shB,
             const collision_context &ctx, collision_result &result);

// Capsule-Box
void collide(const capsule_shape &shA, const box_shape &shB,
             const collision_context &ctx, collision_result &result);

// Box-Capsule
void collide(const box_shape &shA, const capsule_shape &shB,
             const collision_context &ctx, collision_result &result);

// Cylinder-Box
void collide(const cylinder_shape &shA, const box_shape &shB,
             const collision_context &ctx, collision_result &result);

// Box-Cylinder
void collide(const box_shape &shA, const cylinder_shape &shB,
             const collision_context &ctx, collision_result &result);

// Paged Mesh-Paged Mesh
inline
void collide(const paged_mesh_shape &shA, const paged_mesh_shape &shB,
             const collision_context &ctx, collision_result &result) {
    // collision between paged triangle meshes is undefined.
}

// Plane-Paged Mesh
inline
void collide(const plane_shape &shA, const paged_mesh_shape &shB,
             const collision_context &ctx, collision_result &result) {
    // collision between paged triangle meshes and planes is undefined.
}

// Paged Mesh-Plane
inline
void collide(const paged_mesh_shape &shA, const plane_shape &shB,
             const collision_context &ctx, collision_result &result) {
    swap_collide(shA, shB, ctx, result);
}

// Mesh-Paged Mesh
inline
void collide(const mesh_shape &shA, const paged_mesh_shape &shB,
             const collision_context &ctx, collision_result &result) {
    // collision between triangle meshes is undefined.
}

// Paged Mesh-Mesh
inline
void collide(const paged_mesh_shape &shA, const mesh_shape &shB,
             const collision_context &ctx, collision_result &result) {
    swap_collide(shA, shB, ctx, result);
}

// Polyhedron-Polyhedron
void collide(const polyhedron_shape &shA, const polyhedron_shape &shB,
             const collision_context &ctx, collision_result &result);

// Polyhedron-Plane
void collide(const polyhedron_shape &shA, const plane_shape &shB,
             const collision_context &ctx, collision_result &result);

// Plane-Polyhedron
void collide(const plane_shape &shA, const polyhedron_shape &shB,
             const collision_context &ctx, collision_result &result);

// Sphere-Polyhedron
void collide(const sphere_shape &shA, const polyhedron_shape &shB,
             const collision_context &ctx, collision_result &result);

// Polyhedron-Sphere
void collide(const polyhedron_shape &shA, const sphere_shape &shB,
             const collision_context &ctx, collision_result &result);

// Box-Polyhedron
void collide(const box_shape &shA, const polyhedron_shape &shB,
             const collision_context &ctx, collision_result &result);

// Polyhedron-Box
void collide(const polyhedron_shape &shA, const box_shape &shB,
             const collision_context &ctx, collision_result &result);

// Capsule-Polyhedron
void collide(const capsule_shape &shA, const polyhedron_shape &shB,
             const collision_context &ctx, collision_result &result);

// Polyhedron-Capsule
void collide(const polyhedron_shape &shA, const capsule_shape &shB,
             const collision_context &ctx, collision_result &result);

// Cylinder-Polyhedron
void collide(const cylinder_shape &shA, const polyhedron_shape &shB,
             const collision_context &ctx, collision_result &result);

// Polyhedron-Cylinder
void collide(const polyhedron_shape &shA, const cylinder_shape &shB,
             const collision_context &ctx, collision_result &result);

// Compound-Compound
void collide(const compound_shape &shA, const compound_shape &shB,
             const collision_context &ctx, collision_result &result);

// Compound-Plane
void collide(const compound_shape &shA, const plane_shape &shB,
             const collision_context &ctx, collision_result &result);

// Plane-Compound
void collide(const plane_shape &shA, const compound_shape &shB,
             const collision_context &ctx, collision_result &result);

// Compound-Box/Sphere/Cylinder/Capsule/Polyhedron
template<typename T, std::enable_if_t<has_type<T, compound_shape::shapes_variant_t>::value, bool> = true>
void collide(const compound_shape &shA, const T &shB,
             const collision_context &ctx, collision_result &result) {
    // Calculate AABB of B's AABB in A's space.
    auto aabbB_in_A = aabb_to_object_space(ctx.aabbB, ctx.posA, ctx.ornA);
    // A more precise AABB could be obtained but it would be generally more expensive.
    //auto aabbB_in_A = shape_aabb(shB, posB_in_A, ornB_in_A);

    shA.visit(aabbB_in_A, [&] (auto &&sh, auto node_index) {
        auto &nodeA = shA.nodes[node_index];
        // New collision context with A's world space position and orientation.
        auto child_ctx = ctx;
        child_ctx.posA = to_world_space(nodeA.position, ctx.posA, ctx.ornA);
        child_ctx.ornA = ctx.ornA * nodeA.orientation;

        collision_result child_result;
        collide(sh, shB, child_ctx, child_result);

        // The elements of A in the collision points must be transformed from the child
        // node's space into A's space.
        for (size_t i = 0; i < child_result.num_points; ++i) {
            auto &child_point = child_result.point[i];
            child_point.pivotA = to_world_space(child_point.pivotA, nodeA.position, nodeA.orientation);

            if (!child_point.featureA) {
                child_point.featureA = {};
            }

            child_point.featureA->part = node_index;
            result.maybe_add_point(child_point);
        }
    });
}

// Box/Sphere/Cylinder/Capsule/Polyhedron-Compound
template<typename T, std::enable_if_t<has_type<T, compound_shape::shapes_variant_t>::value, bool> = true>
void collide(const T &shA, const compound_shape &shB,
             const collision_context &ctx, collision_result &result) {
    swap_collide(shA, shB, ctx, result);
}

// Box/Sphere/Cylinder/Capsule/Polyhedron/Compound-Mesh
template<typename T>
void collide(const T &shA, const mesh_shape &shB,
             const collision_context &ctx, collision_result &result) {
    collide(shA, *shB.trimesh, ctx, result);
}

// Mesh-Box/Sphere/Cylinder/Capsule/Polyhedron/Compound
template<typename T>
void collide(const mesh_shape &shA, const T &shB,
             const collision_context &ctx, collision_result &result) {
    swap_collide(shA, shB, ctx, result);
}

// Box/Sphere/Cylinder/Capsule/Polyhedron/Compound-Paged Mesh
template<typename T>
void collide(const T &shA, const paged_mesh_shape &shB,
             const collision_context &ctx, collision_result &result) {
    // Inset AABB to load nearby submeshes.
    constexpr auto inset = vector3 {
        -contact_breaking_threshold,
        -contact_breaking_threshold,
        -contact_breaking_threshold
    };
    auto inset_aabb = ctx.aabbA.inset(inset);

    shB.trimesh->visit_submeshes(inset_aabb, [&] (size_t mesh_idx) {
        auto trimesh = shB.trimesh->get_submesh(mesh_idx);
        collision_result child_result;
        collide(shA, *trimesh, ctx, child_result);

        for (size_t i = 0; i < child_result.num_points; ++i) {
            auto &child_point = child_result.point[i];
            child_point.featureB->part = mesh_idx;
            result.maybe_add_point(child_point);
        }
    });
}

// Paged Mesh-Box/Sphere/Cylinder/Capsule/Polyhedron/Compound
template<typename T>
void collide(const paged_mesh_shape &shA, const T &shB,
             const collision_context &ctx, collision_result &result) {
    swap_collide(shA, shB, ctx, result);
}

template<typename ShapeAType, typename ShapeBType>
void swap_collide(const ShapeAType &shA, const ShapeBType &shB,
                  const collision_context &ctx, collision_result &result) {
    collide(shB, shA, ctx.swapped(), result);
    result.swap();
}

#if defined(_MSC_VER)
__pragma(warning(pop))
#endif

}

#endif // EDYN_COLLISION_COLLIDE_HPP
