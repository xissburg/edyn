#ifndef EDYN_MATH_VECTOR2_3_UTIL_HPP
#define EDYN_MATH_VECTOR2_3_UTIL_HPP

#include "edyn/math/vector2.hpp"
#include "edyn/math/vector3.hpp"

namespace edyn {

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

}

#endif // EDYN_MATH_VECTOR2_3_UTIL_HPP
