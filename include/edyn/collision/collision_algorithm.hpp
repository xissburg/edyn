#ifndef EDYN_COLLISION_COLLISION_ALGORITHM_HPP
#define EDYN_COLLISION_COLLISION_ALGORITHM_HPP

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
    return collide(shB, posB, ornB, shA, posA, ornA, threshold).swap();
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
    return collide(shB, posB, ornB, shA, posA, ornA, threshold).swap();
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
    return collide(shB, posB, ornB, shA, posA, ornA, threshold).swap();
}

}

#endif // EDYN_COLLISION_COLLISION_ALGORITHM_HPP