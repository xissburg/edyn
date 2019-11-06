#ifndef EDYN_MATH_VECTOR3_HPP
#define EDYN_MATH_VECTOR3_HPP

#include "scalar.hpp"

namespace edyn {

struct vector3 {
    scalar x, y, z;
};

scalar dot(const vector3& a, const vector3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

}

#endif // EDYN_MATH_VECTOR3_HPP