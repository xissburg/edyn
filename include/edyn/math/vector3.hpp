#ifndef EDYN_MATH_VECTOR3_HPP
#define EDYN_MATH_VECTOR3_HPP

#include <cmath>
#include <algorithm>
#include <type_traits>
#include "edyn/math/scalar.hpp"
#include "edyn/config/config.h"

namespace edyn {

struct vector3 {
    scalar x, y, z;

    constexpr scalar& operator[](size_t i) noexcept {
        EDYN_ASSERT(i < 3);
        return (&x)[i];
    }

    constexpr scalar operator[](size_t i) const noexcept {
        EDYN_ASSERT(i < 3);
        return (&x)[i];
    }

    // Iterate over each vector element.
    template<typename Func>
    void each(Func f) {
        if constexpr(std::is_invocable_v<Func, scalar &, int>) {
            f(x, 0); f(y, 1); f(z, 2);
        } else {
            f(x); f(y); f(z);
        }
    }

    template<typename Func>
    void each(Func f) const {
        if constexpr(std::is_invocable_v<Func, scalar, int>) {
            f(x, 0); f(y, 1); f(z, 2);
        } else {
            f(x); f(y); f(z);
        }
    }
};

// Zero vector.
inline constexpr vector3 vector3_zero {0, 0, 0};

// Vector with all elements set to 1.
inline constexpr vector3 vector3_one {1, 1, 1};

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
constexpr vector3 operator+(const vector3 &v, const vector3 &w) noexcept {
    return {v.x + w.x, v.y + w.y, v.z + w.z};
}

// Add a vector into another vector.
constexpr vector3& operator+=(vector3 &v, const vector3 &w) noexcept {
    v.x += w.x;
    v.y += w.y;
    v.z += w.z;
    return v;
}

// Subtract two vectors.
constexpr vector3 operator-(const vector3 &v, const vector3 &w) noexcept {
    return {v.x - w.x, v.y - w.y, v.z - w.z};
}

// Subtract a vector from another vector.
constexpr vector3& operator-=(vector3 &v, const vector3 &w) noexcept {
    v.x -= w.x;
    v.y -= w.y;
    v.z -= w.z;
    return v;
}

// Negation of a vector.
constexpr vector3 operator-(const vector3 &v) noexcept {
    return {-v.x, -v.y, -v.z};
}

// Multiply vectors component-wise.
constexpr vector3 operator*(const vector3 &v, const vector3 &w) noexcept {
    return {v.x * w.x, v.y * w.y, v.z * w.z};
}

// Divide vectors component-wise.
constexpr vector3 operator/(const vector3 &v, const vector3 &w) noexcept {
    return {v.x / w.x, v.y / w.y, v.z / w.z};
}

// Multiply vector by scalar.
constexpr vector3 operator*(const vector3& v, scalar s) noexcept {
    return {v.x * s, v.y * s, v.z * s};
}

// Multiply scalar by vector.
constexpr vector3 operator*(scalar s, const vector3 &v) noexcept {
    return {s * v.x, s * v.y, s * v.z};
}

// Divide vector by scalar.
constexpr vector3 operator/(const vector3 &v, scalar s) noexcept {
    return {v.x / s, v.y / s, v.z / s};
}

// Divide scalar by vector.
constexpr vector3 operator/(scalar s, const vector3 &v) noexcept {
    return {s / v.x, s / v.y, s / v.z};
}

// Scale a vector.
constexpr vector3& operator*=(vector3 &v, scalar s) noexcept {
    v.x *= s;
    v.y *= s;
    v.z *= s;
    return v;
}

// Inverse-scale a vector.
constexpr vector3& operator/=(vector3 &v, scalar s) noexcept {
    auto z = scalar(1) / s;
    v.x *= z;
    v.y *= z;
    v.z *= z;
    return v;
}

// Multiply vectors component-wise and assign to the first.
constexpr vector3& operator*=(vector3 &v, const vector3 &w) noexcept {
    v.x *= w.x;
    v.y *= w.y;
    v.z *= w.z;
    return v;
}

// Divide vectors component-wise and assign to the first.
constexpr vector3& operator/=(vector3 &v, const vector3 &w) noexcept {
    v.x /= w.x;
    v.y /= w.y;
    v.z /= w.z;
    return v;
}

// Check if two vectors are equal.
constexpr bool operator==(const vector3 &v, const vector3 &w) noexcept {
    return v.x == w.x && v.y == w.y && v.z == w.z;
}

// Check if two vectors are different.
constexpr bool operator!=(const vector3 &v, const vector3 &w) noexcept {
    return v.x != w.x || v.y != w.y || v.z != w.z;
}

// Check if a vector is bigger than another component-wise.
constexpr bool operator>(const vector3 &v, const vector3 &w) noexcept {
    return v.x > w.x && v.y > w.y && v.z > w.z;
}

// Check if a vector is smaller than another component-wise.
constexpr bool operator<(const vector3 &v, const vector3 &w) noexcept {
    return v.x < w.x && v.y < w.y && v.z < w.z;
}

// Check if a vector is greater than or equal to another component-wise.
constexpr bool operator>=(const vector3 &v, const vector3 &w) noexcept {
    return v.x >= w.x && v.y >= w.y && v.z >= w.z;
}

// Check if a vector is less than or equal to another component-wise.
constexpr bool operator<=(const vector3 &v, const vector3 &w) noexcept {
    return v.x <= w.x && v.y <= w.y && v.z <= w.z;
}

// Dot product between vectors.
constexpr scalar dot(const vector3 &v, const vector3 &w) noexcept {
    return v.x * w.x + v.y * w.y + v.z * w.z;
}

// Cross product between two vectors.
constexpr vector3 cross(const vector3 &v, const vector3 &w) noexcept {
    return {v.y * w.z - v.z * w.y,
            v.z * w.x - v.x * w.z,
            v.x * w.y - v.y * w.x};
}

// Triple product among three vectors, i.e. the dot product of one of
// them with the cross product of the other two.
constexpr scalar triple_product(const vector3 &u,
                             const vector3 &v,
                             const vector3 &w) noexcept {
    return dot(u, cross(v, w));
}

// Square length of a vector.
constexpr scalar length_sqr(const vector3 &v) noexcept {
    return dot(v, v);
}

// Length of a vector.
inline scalar length(const vector3 &v) noexcept {
    return std::sqrt(length_sqr(v));
}

// Distance between two points.
inline scalar distance(const vector3 &p0, const vector3 &p1) noexcept {
    return length(p0 - p1);
}

// Squared distance between two points.
constexpr scalar distance_sqr(const vector3 &p0, const vector3 &p1) noexcept {
    return length_sqr(p0 - p1);
}

// Normalized vector (unit length). Asserts if the vector's length is zero.
inline vector3 normalize(const vector3 &v) noexcept {
    auto l = length(v);
    EDYN_ASSERT(l > 1e-18);
    return v / l;
}

// Normalizes vector if it's length is greater than a threshold above zero.
// Returns true if the vector was normalized.
inline bool try_normalize(vector3 &v) noexcept {
    auto lsqr = length_sqr(v);

    if (lsqr > 1e-18) {
        v /= std::sqrt(lsqr);
        return true;
    }

    return false;
}

// Projects direction vector `v` onto plane with normal `n`.
constexpr vector3 project_direction(const vector3 &v, const vector3 &n) noexcept {
    return v - n * dot(v, n);
}

// Projects point `p` onto plane with origin `q` and normal `n`.
constexpr vector3 project_plane(const vector3 &p, const vector3 &q, const vector3 &n) noexcept {
    return p - n * dot(p - q, n);
}

// Performs element-wise minimum.
constexpr vector3 min(const vector3 &v, const vector3 &w) noexcept {
    return {std::min(v.x, w.x), std::min(v.y, w.y), std::min(v.z, w.z)};
}

// Performs element-wise maximum.
constexpr vector3 max(const vector3 &v, const vector3 &w) noexcept {
    return {std::max(v.x, w.x), std::max(v.y, w.y), std::max(v.z, w.z)};
}

// Performs element-wise absolute.
inline vector3 abs(const vector3 &v) noexcept {
    return {std::abs(v.x), std::abs(v.y), std::abs(v.z)};
}

// Returns the index of the coordinate with lowest value.
constexpr size_t min_index(const vector3 &v) noexcept {
    auto min_val = v.x;
    size_t min_idx = 0;

    if (v.y < min_val) {
        min_val = v.y;
        min_idx = 1;
    }

    if (v.z < min_val) {
        min_idx = 2;
    }

    return min_idx;
}

// Returns the index of the coordinate with lowest absolute value.
inline size_t min_index_abs(const vector3 &v) noexcept {
    return min_index(abs(v));
}

// Returns the index of the coordinate with greatest value.
constexpr size_t max_index(const vector3 &v) noexcept {
    auto max_val = v.x;
    size_t max_idx = 0;

    if (v.y > max_val) {
        max_val = v.y;
        max_idx = 1;
    }

    if (v.z > max_val) {
        max_idx = 2;
    }

    return max_idx;
}

// Returns the index of the coordinate with greatest absolute value.
inline size_t max_index_abs(const vector3 &v) noexcept {
    return max_index(abs(v));
}

// Apply the `floor` function to all elements.
inline vector3 floor(const vector3 &v) noexcept {
    return {std::floor(v.x), std::floor(v.y), std::floor(v.z)};
}

// Apply the `ceil` function to all elements.
inline vector3 ceil(const vector3 &v) noexcept {
    return {std::ceil(v.x), std::ceil(v.y), std::ceil(v.z)};
}

}

#endif // EDYN_MATH_VECTOR3_HPP
