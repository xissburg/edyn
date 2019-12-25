#ifndef EDYN_MATH_MATH_HPP
#define EDYN_MATH_MATH_HPP

#include "constants.hpp"
#include <algorithm>

namespace edyn {

/**
 * @return Value of `radians` converted to degrees.
 */
inline scalar to_degrees(scalar radians) {
    return radians / pi * 180;
}

/**
 * @return Value of `degress` converted to radians.
 */
inline scalar to_radians(scalar degrees) {
    return degrees / 180 * pi;
}

/**
 * @return Scalar clamped to the [0, 1] interval.
 */
inline scalar clamp_unit(scalar s) {
    return std::clamp(s, scalar(0), scalar(1));
}

}

#endif // EDYN_MATH_MATH_HPP