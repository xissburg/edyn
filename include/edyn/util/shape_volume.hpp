#ifndef EDYN_UTIL_SHAPE_VOLUME_HPP
#define EDYN_UTIL_SHAPE_VOLUME_HPP

#include "edyn/math/vector3.hpp"

namespace edyn {

scalar cylinder_volume(scalar radius, scalar length);
scalar sphere_volume(scalar radius);
scalar box_volume(const vector3 &extents);

struct box_shape;
struct capsule_shape;
struct compound_shape;
struct cylinder_shape;
struct polyhedron_shape;
struct sphere_shape;

scalar shape_volume(const box_shape &sh);
scalar shape_volume(const capsule_shape &sh);
scalar shape_volume(const compound_shape &sh);
scalar shape_volume(const cylinder_shape &sh);
scalar shape_volume(const polyhedron_shape &sh);
scalar shape_volume(const sphere_shape &sh);

}

#endif // EDYN_UTIL_SHAPE_VOLUME_HPP
