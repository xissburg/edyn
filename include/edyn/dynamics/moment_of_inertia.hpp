#ifndef EDYN_DYNAMICS_MOMENT_OF_INERTIA_HPP
#define EDYN_DYNAMICS_MOMENT_OF_INERTIA_HPP

#include <vector>
#include <cstdint>
#include "edyn/math/vector3.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/shapes/shapes.hpp"

namespace edyn {

vector3 moment_of_inertia_solid_box(scalar mass, const vector3 &extents);
vector3 moment_of_inertia_solid_capsule(scalar mass, scalar len, scalar radius, coordinate_axis);
scalar moment_of_inertia_solid_sphere(scalar mass, scalar radius);
scalar moment_of_inertia_hollow_sphere(scalar mass, scalar radius);

/**
 * @brief Calculates the diagonal of the inertia tensor for a solid cylinder.
 * @param mass Mass of cylinder in kilograms.
 * @param len Length of cylinder along its axis.
 * @param radius Radius of cylinder.
 * @param axis Main axis of cylinder.
 * @return Diagonal of inertia tensor.
 */
vector3 moment_of_inertia_solid_cylinder(scalar mass, scalar len, scalar radius,
                                         coordinate_axis axis);

/**
 * @brief Calculates the diagonal of the inertia tensor for a hollow cylinder.
 * @param mass Mass of cylinder in kilograms.
 * @param len Length of cylinder along its axis.
 * @param inner_radius Inner radius of cylinder where there's no material.
 * @param outer_radius Material exists inbetween the inner and outer radii of
 * the cylinder.
 * @param axis Main axis of cylinder.
 * @return Diagonal of inertia tensor.
 */
vector3 moment_of_inertia_hollow_cylinder(scalar mass, scalar len,
                                          scalar inner_radius, scalar outer_radius,
                                          coordinate_axis axis);

matrix3x3 moment_of_inertia_polyhedron(scalar mass,
                                       const std::vector<vector3> &vertices,
                                       const std::vector<uint32_t> &indices,
                                       const std::vector<uint32_t> &faces);

// Default moment of inertia for shapes.
matrix3x3 moment_of_inertia(const plane_shape &sh, scalar mass);
matrix3x3 moment_of_inertia(const sphere_shape &sh, scalar mass);
matrix3x3 moment_of_inertia(const cylinder_shape &sh, scalar mass);
matrix3x3 moment_of_inertia(const capsule_shape &sh, scalar mass);
matrix3x3 moment_of_inertia(const mesh_shape &sh, scalar mass);
matrix3x3 moment_of_inertia(const box_shape &sh, scalar mass);
matrix3x3 moment_of_inertia(const polyhedron_shape &sh, scalar mass);
matrix3x3 moment_of_inertia(const compound_shape &sh, scalar mass);
matrix3x3 moment_of_inertia(const paged_mesh_shape &sh, scalar mass);

/**
 * @brief Visits the shape variant and calculates the moment of inertia of the
 * shape it holds.
 * @param var The shape variant.
 * @param mass Shape's mass.
 * @return Inertia tensor.
 */
matrix3x3 moment_of_inertia(const shapes_variant_t &var, scalar mass);

/**
 * @brief Calculate the moment of inertia about a different location.
 * @param inertia Moment of inertia about centroid.
 * @param mass Rigid body mass.
 * @param offset Offset from centroid.
 * @return Moment of inertia at the requested offset from centroid.
 */
matrix3x3 shift_moment_of_inertia(const matrix3x3 &inertia, scalar mass, const vector3 &offset);

}

#endif // EDYN_DYNAMICS_MOMENT_OF_INERTIA_HPP
