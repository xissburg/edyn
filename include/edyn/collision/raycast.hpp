#ifndef EDYN_COLLISION_RAYCAST_HPP
#define EDYN_COLLISION_RAYCAST_HPP

#include <variant>
#include <entt/entity/registry.hpp>
#include "edyn/math/vector3.hpp"
#include "edyn/shapes/cylinder_shape.hpp"
#include "edyn/shapes/capsule_shape.hpp"
#include "edyn/util/tuple_util.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/origin.hpp"
#include "edyn/comp/shape_index.hpp"
#include "edyn/collision/broadphase_worker.hpp"
#include "edyn/shapes/shapes.hpp"

namespace edyn {

struct box_shape;
struct sphere_shape;
struct polyhedron_shape;
struct compound_shape;
struct plane_shape;
struct mesh_shape;
struct paged_mesh_shape;

/**
 * @brief Info provided when raycasting a box.
 */
struct box_raycast_info {
    // Index of face the ray intersects.
    size_t face_index;
};

/**
 * @brief Info provided when raycasting a cylinder.
 */
struct cylinder_raycast_info {
    // Feature the ray intersects. Either a face or side edge.
    cylinder_feature feature;
    // If the feature is a face, this is the index.
    size_t face_index;
};

/**
 * @brief Info provided when raycasting a capsule.
 */
struct capsule_raycast_info {
    // Feature the ray intersects. Either a hemisphere or side.
    capsule_feature feature;
    // If the feature is a hemisphere, this is the index.
    size_t hemisphere_index;
};

/**
 * @brief Info provided when raycasting a polyhedron.
 */
struct polyhedron_raycast_info {
    // Index of face the ray intersects.
    size_t face_index;
};

/**
 * @brief Info provided when raycasting a triangle mesh.
 */
struct mesh_raycast_info {
    // Index of triangle the ray intersects.
    size_t triangle_index;
};

/**
 * @brief Info provided when raycasting a paged triangle mesh.
 */
struct paged_mesh_raycast_info {
    // Index of submesh where the intersected triangle is located.
    size_t submesh_index;
    // Index of intersected triangle in the submesh.
    size_t triangle_index;
};

/**
 * @brief Info provided when raycasting a compound.
 */
struct compound_raycast_info {
    // Index of child shape.
    size_t child_index;
    // Raycast info for child shape if any extra info is available.
    std::variant<
        std::monostate,
        box_raycast_info,
        cylinder_raycast_info,
        capsule_raycast_info,
        polyhedron_raycast_info
    > child_info_var;
};

/**
 * @brief Information returned from a shape-specific raycast query.
 */
struct shape_raycast_result {
    // Fraction for the ray where intersection occurs. The intersection
    // point is at `lerp(p0, p1, fraction)`.
    scalar fraction { EDYN_SCALAR_MAX };
    // Normal vector at intersection.
    vector3 normal;
    // Raycast details which contains a value that depends on the type of shape
    // that was hit.
    std::variant<
        std::monostate,
        box_raycast_info,
        cylinder_raycast_info,
        capsule_raycast_info,
        polyhedron_raycast_info,
        compound_raycast_info,
        mesh_raycast_info,
        paged_mesh_raycast_info
    > info_var;
};

/**
 * @brief Information returned from a general raycast query.
 * It derives from `shape_raycast_result` thus bringing in the extra details of
 * the raycast as well.
 */
struct raycast_result : public shape_raycast_result {
    // The entity that was hit. It's set to `entt::null` if no entity is hit.
    entt::entity entity { entt::null };
};

/**
 * @brief Input for a shape-specific raycast query containing the spatial
 * configuration.
 */
struct raycast_context {
    // Position of shape.
    vector3 pos;
    // Orientation of shape.
    quaternion orn;
    // First point in the ray.
    vector3 p0;
    // Second point in the ray.
    vector3 p1;
};

/**
 * @brief Performs a raycast query on a registry.
 * @param registry Data source.
 * @param p0 First point in the ray.
 * @param p1 Second point in the ray.
 * @param ignore_func Function that returns whether an entity should be ignored.
 * @return Result.
 */
template<typename IgnoreFunc>
raycast_result raycast(entt::registry &registry, vector3 p0, vector3 p1,
                       IgnoreFunc ignore_func) {
    auto index_view = registry.view<shape_index>();
    auto tr_view = registry.view<position, orientation>();
    auto origin_view = registry.view<origin>();
    auto shape_views_tuple = get_tuple_of_shape_views(registry);

    entt::entity hit_entity {entt::null};
    shape_raycast_result result;

    auto raycast_shape = [&](entt::entity entity) {
        if (ignore_func(entity)) {
            return;
        }

        auto sh_idx = index_view.get<shape_index>(entity);
        auto pos = origin_view.contains(entity) ? static_cast<vector3>(origin_view.get<origin>(entity)) : tr_view.get<position>(entity);
        auto orn = tr_view.get<orientation>(entity);
        auto ctx = raycast_context{pos, orn, p0, p1};

        visit_shape(sh_idx, entity, shape_views_tuple, [&](auto &&shape) {
            auto res = shape_raycast(shape, ctx);

            if (res.fraction < result.fraction) {
                result = res;
                hit_entity = entity;
            }
        });
    };

    // This function works both in the coordinator and in an island worker.
    // Pick the available broadphase and raycast their AABB trees.
    /* if (registry.ctx().find<broadphase_main>() != nullptr) {
        auto &bphase = registry.ctx().at<broadphase_main>();
        bphase.raycast_islands(p0, p1, [&](entt::entity island_entity) {
            auto &tree_view = tree_view_view.get<edyn::tree_view>(island_entity);
            tree_view.raycast(p0, p1, [&](tree_node_id_t id) {
                auto entity = tree_view.get_node(id).entity;
                raycast_shape(entity);
            });
        });

        bphase.raycast_non_procedural(p0, p1, raycast_shape);
    } else {
        auto &bphase = registry.ctx().at<broadphase_worker>();
        bphase.raycast(p0, p1, raycast_shape);
    } */

    return {result, hit_entity};
}

/*! @copydoc raycast */
inline raycast_result raycast(entt::registry &registry, vector3 p0, vector3 p1) {
    return raycast(registry, p0, p1, [](auto) { return false; });
}

// Raycast functions for each shape.

shape_raycast_result shape_raycast(const box_shape &, const raycast_context &);
shape_raycast_result shape_raycast(const cylinder_shape &, const raycast_context &);
shape_raycast_result shape_raycast(const sphere_shape &, const raycast_context &);
shape_raycast_result shape_raycast(const capsule_shape &, const raycast_context &);
shape_raycast_result shape_raycast(const polyhedron_shape &, const raycast_context &);
shape_raycast_result shape_raycast(const compound_shape &, const raycast_context &);
shape_raycast_result shape_raycast(const plane_shape &, const raycast_context &);
shape_raycast_result shape_raycast(const mesh_shape &, const raycast_context &);
shape_raycast_result shape_raycast(const paged_mesh_shape &, const raycast_context &);

}

#endif // EDYN_COLLISION_RAYCAST_HPP
