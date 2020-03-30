#ifndef EDYN_MATH_VECTOR3_HPP
#define EDYN_MATH_VECTOR3_HPP

#include <cmath>
#include "scalar.hpp"
#include "edyn/config/config.h"

namespace edyn {

struct vector3 {
    scalar x, y, z;

    scalar& operator[](size_t i) {
        EDYN_ASSERT(i < 3);
        return (&x)[i];
    }

    scalar operator[](size_t i) const {
        EDYN_ASSERT(i < 3);
        return (&x)[i];
    }
};

// Zero vector.
inline constexpr vector3 vector3_zero {0, 0, 0};

// Unit vector pointing in the x direction.
inline constexpr vector3 vector3_x {1, 0, 0};

// Unit vector pointing in the y direction.
inline constexpr vector3 vector3_y {0, 1, 0};

// Unit vector pointing in the z direction.
inline constexpr vector3 vector3_z {0, 0, 1};

// Vector with minumum values.
inline constexpr vector3 vector3_min {EDYN_SCALAR_MIN, EDYN_SCALAR_MIN, EDYN_SCALAR_MIN};

// Vector with maximum values.
inline constexpr vector3 vector3_max {EDYN_SCALAR_MAX, EDYN_SCALAR_MAX, EDYN_SCALAR_MAX};

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
inline vector3 operator-(const vector3 &v) {
    return {-v.x, -v.y, -v.z};
}

// Multiply vectors component-wise.
inline vector3 operator*(const vector3 &v, const vector3 &w) {
    return {v.x * w.x, v.y * w.y, v.z * w.z};
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
    auto z = scalar(1) / s;
    v.x *= z;
    v.y *= z;
    v.z *= z;
    return v;
}

// Check if two vectors are equal.
inline bool operator==(const vector3 &v, const vector3 &w) {
    return v.x == w.x && v.y == w.y && v.z == w.z;
}

// Check if two vectors are different.
inline bool operator!=(const vector3 &v, const vector3 &w) {
    return v.x != w.x || v.y != w.y || v.z != w.z;
}

// Check if a vector is bigger than another component-wise.
inline bool operator>(const vector3 &v, const vector3 &w) {
    return v.x > w.x && v.y > w.y && v.z > w.z;
}

// Check if a vector is smaller than another component-wise.
inline bool operator<(const vector3 &v, const vector3 &w) {
    return v.x < w.x && v.y < w.y && v.z < w.z;
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

// Distance between two points.
inline scalar distance(const vector3 &p0, const vector3 &p1) {
    return length(p0 - p1);
}

// Squared distance between two points.
inline scalar distance2(const vector3 &p0, const vector3 &p1) {
    return length2(p0 - p1);
}

// Normalized vector (unit length). Asserts if the vector's length is zero.
inline vector3 normalize(const vector3 &v) {
    auto l = length(v);
    EDYN_ASSERT(l > EDYN_EPSILON);
    return v / l;
}

// Projects direction vector `v` onto plane with normal `n`.
inline vector3 project_direction(const vector3 &v, const vector3 &n) {
    return v - n * dot(v, n);
}

// Projects point `p` onto plane with origin `q` and normal `n`.
inline vector3 project_plane(const vector3 &p, const vector3 &q, const vector3 &n) {
    return p - n * dot(p - q, n);
}

inline vector3 min(const vector3 &v, const vector3 &w) {
    return {std::min(v.x, w.x), std::min(v.y, w.y), std::min(v.z, w.z)};
}

inline vector3 max(const vector3 &v, const vector3 &w) {
    return {std::max(v.x, w.x), std::max(v.y, w.y), std::max(v.z, w.z)};
}

}

#endif // EDYN_MATH_VECTOR3_HPP