#ifndef EDYN_MATH_MATH_HPP
#define EDYN_MATH_MATH_HPP

#include "constants.hpp"

namespace edyn {

inline scalar to_degrees(scalar radians) {
    return radians / pi * 180;
}

inline scalar to_radians(scalar degrees) {
    return degrees / 180 * pi;
}

}

#endif // EDYN_MATH_MATH_HPP