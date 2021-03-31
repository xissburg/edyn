#ifndef EDYN_UTIL_MOMENT_OF_INERTIA_HPP
#define EDYN_UTIL_MOMENT_OF_INERTIA_HPP

#include <vector>
#include <cstdint>
#include "edyn/math/vector3.hpp"
#include "edyn/math/matrix3x3.hpp"

namespace edyn {

vector3 moment_of_inertia_solid_box(scalar mass, const vector3 &extents);
vector3 moment_of_inertia_solid_capsule(scalar mass, scalar len, scalar radius);
vector3 moment_of_inertia_solid_sphere(scalar mass, scalar radius);
vector3 moment_of_inertia_hollow_sphere(scalar mass, scalar radius);

/**
 * @brief Calculates the diagonal of the inertia tensor for a solid cylinder
 * aligned with the x axis.
 * @param mass Mass of cylinder in kilograms.
 * @param len Length of cylinder along its axis.
 * @param radius Radius of cylinder.
 * @return Diagonal of inertia tensor.
 */
vector3 moment_of_inertia_solid_cylinder(scalar mass, scalar len, scalar radius);

/**
 * @brief Calculates the diagonal of the inertia tensor for a hollow cylinder
 * aligned with the x axis.
 * @param mass Mass of cylinder in kilograms.
 * @param len Length of cylinder along its axis.
 * @param inner_radius Inner radius of cylinder where there's no material.
 * @param outer_radius Material exists inbetween the inner and outer radii of
 * the cylinder.
 * @return Diagonal of inertia tensor.
 */
vector3 moment_of_inertia_hollow_cylinder(scalar mass, scalar len, 
                                          scalar inner_radius, scalar outer_radius);

matrix3x3 moment_of_inertia_polyhedron(scalar mass, 
                                       const std::vector<vector3> &vertices, 
                                       const std::vector<uint16_t> &indices,
                                       const std::vector<uint16_t> &faces);

}

#endif // EDYN_UTIL_MOMENT_OF_INERTIA_HPP
