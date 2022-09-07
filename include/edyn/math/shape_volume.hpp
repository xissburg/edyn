#ifndef EDYN_MATH_SHAPE_VOLUME_HPP
#define EDYN_MATH_SHAPE_VOLUME_HPP

#include "edyn/math/vector3.hpp"

namespace edyn {

struct box_shape;
struct capsule_shape;
struct compound_shape;
struct cylinder_shape;
struct polyhedron_shape;
struct sphere_shape;
struct convex_mesh;

/**
 * @brief Calculate volume of a cylinder.
 * @param radius Radius of cylinder.
 * @param length Length of cylinder.
 * @return Volume of cylinder.
 */
scalar cylinder_volume(scalar radius, scalar length);

/**
 * @brief Calculate volume of a sphere.
 * @param radius Radius of sphere.
 * @return Volume of sphere.
 */
scalar sphere_volume(scalar radius);

/**
 * @brief Calculate volume of a box.
 * @param extents Size of box in all 3 dimensions.
 * @return Volume of box.
 */
scalar box_volume(const vector3 &extents);

/**
 * @brief Calculate volume of a convex mesh.
 * @param mesh A convex mesh.
 * @return Volume of convex mesh.
 */
scalar mesh_volume(const convex_mesh &mesh);

// Calculate volume for all relevant shapes.
scalar shape_volume(const box_shape &sh);
scalar shape_volume(const capsule_shape &sh);
scalar shape_volume(const compound_shape &sh);
scalar shape_volume(const cylinder_shape &sh);
scalar shape_volume(const polyhedron_shape &sh);
scalar shape_volume(const sphere_shape &sh);

}

#endif // EDYN_MATH_SHAPE_VOLUME_HPP
