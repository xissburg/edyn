#ifndef EDYN_MATH_VECTOR3_HPP
#define EDYN_MATH_VECTOR3_HPP

#include "scalar.hpp"

namespace edyn {

struct vector3 {
    scalar x, y, z;

    vector3 operator+(const vector3 &rhs) const {
        return {x + rhs.x, y + rhs.y, z + rhs.z};
    }

    vector3 operator-(const vector3 &rhs) const {
        return {x - rhs.x, y - rhs.y, z - rhs.z};
    }
    
    vector3 operator-() const {
        return {-x, -y, -z};
    }

    vector3 operator*(scalar rhs) const {
        return {x * rhs, y * rhs, z * rhs};
    }
    
    friend vector3 operator*(scalar lhs, const vector3 &rhs) {
        return {lhs * rhs.x, lhs * rhs.y, lhs * rhs.z};
    }
    
    vector3 operator/(scalar rhs) const {
        return {x / rhs, y / rhs, z / rhs};
    }
    
    friend vector3 operator/(scalar lhs, const vector3 &rhs) {
        return {lhs / rhs.x, lhs / rhs.y, lhs / rhs.z};
    }
};

scalar dot(const vector3 &a, const vector3 &b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

}

#endif // EDYN_MATH_VECTOR3_HPP