#ifndef EDYN_MATH_MATH_HPP
#define EDYN_MATH_MATH_HPP

#include "constants.hpp"

namespace edyn {

scalar to_degrees(scalar radians) {
    return radians / pi * 180;
}

scalar to_radians(scalar degrees) {
    return degrees / 180 * pi;
}

}

#endif // EDYN_MATH_MATH_HPP