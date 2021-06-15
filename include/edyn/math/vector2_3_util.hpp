#ifndef EDYN_MATH_VECTOR2_3_UTIL_HPP
#define EDYN_MATH_VECTOR2_3_UTIL_HPP

#include "edyn/math/vector2.hpp"
#include "edyn/math/vector3.hpp"

namespace edyn {

// Series of functions to project a `vector3` into a coordinate plane and 
// return that as a `vector2`.

inline vector2 to_vector2_xy(const vector3 &v) {
    return {v.x, v.y};
}

inline vector2 to_vector2_xz(const vector3 &v) {
    return {v.x, v.z};
}

inline vector2 to_vector2_yz(const vector3 &v) {
    return {v.y, v.z};
}

inline vector2 to_vector2_zy(const vector3 &v) {
    return {v.z, v.y};
}

inline vector2 to_vector2_yx(const vector3 &v) {
    return {v.y, v.x};
}

inline vector2 to_vector2_zx(const vector3 &v) {
    return {v.z, v.x};
}

// Series of functions to convert a `vector2` into a `vector3` contained in a
// coordinate plane.

inline vector3 to_vector3_xy(const vector2 &v) {
    return {v.x, v.y, 0};
}

inline vector3 to_vector3_xz(const vector2 &v) {
    return {v.x, 0, v.y};
}

inline vector3 to_vector3_yz(const vector2 &v) {
    return {0, v.x, v.y};
}

inline vector3 to_vector3_zy(const vector2 &v) {
    return {0, v.y, v.x};
}

inline vector3 to_vector3_yx(const vector2 &v) {
    return {v.y, v.x, 0};
}

inline vector3 to_vector3_zx(const vector2 &v) {
    return {v.y, 0, v.x};
}

}

#endif // EDYN_MATH_VECTOR2_3_UTIL_HPP
