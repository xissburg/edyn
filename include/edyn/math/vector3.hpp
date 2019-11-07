#ifndef EDYN_MATH_VECTOR3_HPP
#define EDYN_MATH_VECTOR3_HPP

#include <cmath>
#include "scalar.hpp"
#include "../config/config.h"

namespace edyn {

struct vector3 {
    scalar x, y, z;
};

// Zero vector.
inline constexpr vector3 vector3_zero {0, 0, 0};

// Unit vector pointing in the x direction.
inline constexpr vector3 vector3_x {1, 0, 0};

// Unit vector pointing in the y direction.
inline constexpr vector3 vector3_y {0, 1, 0};

// Unit vector pointing in the z direction.
inline constexpr vector3 vector3_z {0, 0, 1};

// Add two vectors.
inline vector3 operator+(const vector3 &v, const vector3 &w) {
    return {v.x + w.x, v.y + w.y, v.z + w.z};
}

// Add a vector into another vector.
inline vector3& operator+=(vector3 &v, const vector3 &w) {
    v.x += w.x;
    v.y += w.y;
    v.z += w.z;
    return v;
}

// Subtract two vectors.
inline vector3 operator-(const vector3 &v, const vector3 &w) {
    return {v.x - w.x, v.y - w.y, v.z - w.z};
}

// Subtract a vector from another vector.
inline vector3& operator-=(vector3 &v, const vector3 &w) {
    v.x -= w.x;
    v.y -= w.y;
    v.z -= w.z;
    return v;
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

// Scale a vector.
inline vector3& operator*=(vector3 &v, scalar s) {
    v.x *= s;
    v.y *= s;
    v.z *= s;
    return v;
}

// Inverse-scale a vector.
inline vector3& operator/=(vector3 &v, scalar s) {
    v.x /= s;
    v.y /= s;
    v.z /= s;
    return v;
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

// Cross product between two vectors.
inline vector3 cross(const vector3 &v, const vector3 &w) {
    return {v.y * w.z - v.z * w.y,
            v.z * w.x - v.x * w.z,
            v.x * w.y - v.y * w.x};
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