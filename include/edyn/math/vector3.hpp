#ifndef EDYN_MATH_VECTOR3_HPP
#define EDYN_MATH_VECTOR3_HPP

#include "scalar.hpp"
#include "../config/config.h"

namespace edyn {

struct vector3 {
    scalar x, y, z;

    // Add another vector to this.
    inline vector3& operator+=(const vector3 &v) {
        x += v.x;
        y += v.y;
        z += v.z;
        return *this;
    }

    // Subtract a vector from this.
    inline vector3& operator-=(const vector3 &v) {
        x -= v.x;
        y -= v.y;
        z -= v.z;
        return *this;
    }

    // Scale this vector.
    inline vector3& operator*=(scalar s) {
        x *= s;
        y *= s;
        z *= s;
        return *this;
    }

    // Inverse-scale this vector.
    inline vector3& operator/=(scalar s) {
        x /= s;
        y /= s;
        z /= s;
        return *this;
    }

    // Zero vector.
    static const vector3 zero;

    // Unit vector pointing in the x direction.
    static const vector3 unit_x;

    // Unit vector pointing in the y direction.
    static const vector3 unit_y;

    // Unit vector pointing in the z direction.
    static const vector3 unit_z;
};

inline constexpr const vector3 vector3::zero {0, 0, 0};
inline constexpr const vector3 vector3::unit_x {1, 0, 0};
inline constexpr const vector3 vector3::unit_y {0, 1, 0};
inline constexpr const vector3 vector3::unit_z {0, 0, 1};

// Add two vectors.
inline vector3 operator+(const vector3 &v, const vector3 &w) {
    return {v.x + w.x, v.y + w.y, v.z + w.z};
}

// Subtract two vectors.
inline vector3 operator-(const vector3 &v, const vector3 &w) {
    return {v.x - w.x, v.y - w.y, v.z - w.z};
}

// Negation of a vector.
inline vector3 operator-(const vector3& v) {
    return {-v.x, -v.y, -v.z};
}

// Multiply vector by scalar.
inline vector3 operator*(const vector3& v, scalar s) {
    return {v.x * s, v.y * s, v.z * s};
}

// Multiply scalar by vector.
inline vector3 operator*(scalar s, const vector3 &v) {
    return {s * v.x, s * v.y, s * v.z};
}

// Divide vector by scalar.
inline vector3 operator/(const vector3 &v, scalar s) {
    return {v.x / s, v.y / s, v.z / s};
}

// Divide scalar by vector.
inline vector3 operator/(scalar s, const vector3 &v) {
    return {s / v.x, s / v.y, s / v.z};
}

// Check if two vectors are equal.
inline bool operator==(const vector3 &v, const vector3 &w) {
    return v.x == w.x && v.y == w.y && v.z == w.z;
}

// Check if two vectors are different.
inline bool operator!=(const vector3 &v, const vector3 &w) {
    return v.x != w.x && v.y != w.y && v.z != w.z;
}

// Dot product between vectors.
inline scalar dot(const vector3 &v, const vector3 &w) {
    return v.x * w.x + v.y * w.y + v.z * w.z;
}

// Square length of a vector.
inline scalar length2(const vector3 &v) {
    return dot(v, v);
}

// Length of a vector.
inline scalar length(const vector3 &v) {
    return std::sqrt(length2(v));
}

// Normalized vector (unit length). Asserts if the vector's length is zero.
inline vector3 normalize(const vector3 &v) {
    auto l = length(v);
    EDYN_ASSERT(l > EDYN_EPSILON);
    return v / l;
}

}

#endif // EDYN_MATH_VECTOR3_HPP