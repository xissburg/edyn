#ifndef EDYN_MATH_QUATERNION_HPP
#define EDYN_MATH_QUATERNION_HPP

#include "vector3.hpp"

namespace edyn {

struct quaternion {
    scalar x, y, z, w;
};

inline constexpr quaternion quaternion_identity {0, 0, 0, 1};

}

#endif // EDYN_MATH_QUATERNION_HPP