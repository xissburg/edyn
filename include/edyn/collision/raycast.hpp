#ifndef EDYN_COLLISION_RAYCAST_HPP
#define EDYN_COLLISION_RAYCAST_HPP

#include <entt/signal/fwd.hpp>
#include <limits>
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
#include "edyn/collision/broadphase.hpp"
#include "edyn/shapes/shapes.hpp"
#include "edyn/util/vector_util.hpp"

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

    raycast_result & operator=(const shape_raycast_result &result) {
        shape_raycast_result::operator=(result);
        return *this;
    }
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

using raycast_id_type = unsigned;
static constexpr auto invalid_raycast_id = std::numeric_limits<raycast_id_type>::max();
using raycast_delegate_type = entt::delegate<void(raycast_id_type, const raycast_result &, vector3, vector3)>;

/**
 * @brief Performs a raycast against all rigid bodies. Do not call this if Edyn
 * was initialized with `execution_mode::asynchronous`, use `raycast_async`
 * instead.
 * @param registry Data source.
 * @param p0 First point in the ray.
 * @param p1 Second point in the ray.
 * @param ignore_entities Entities to be ignored during raycast.
 * @return Result containing the first entity that was hit by the ray.
 */
raycast_result raycast(entt::registry &registry, vector3 p0, vector3 p1,
                       const std::vector<entt::entity> &ignore_entities = {});

/**
 * @brief Performs a raycast query asynchronously. Only call this function if
 * Edyn was initialized in `execution_mode::asynchronous`.
 * @param registry Data source.
 * @param p0 First point in the ray.
 * @param p1 Second point in the ray.
 * @param delegate Triggered when the results are available.
 * @param ignore_entities Entities to be ignored during raycast.
 * @return Request id, which will be passed to the delegate when it is invoked.
 */
raycast_id_type raycast_async(entt::registry &registry, vector3 p0, vector3 p1,
                              const raycast_delegate_type &delegate,
                              const std::vector<entt::entity> &ignore_entities = {});

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
