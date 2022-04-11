#ifndef EDYN_MATH_VECTOR2_HPP
#define EDYN_MATH_VECTOR2_HPP

#include <cmath>
#include "edyn/math/scalar.hpp"
#include "edyn/config/config.h"

namespace edyn {

struct vector2 {
    scalar x, y;

    scalar& operator[](size_t i) noexcept {
        EDYN_ASSERT(i < 2);
        return (&x)[i];
    }

    scalar operator[](size_t i) const noexcept {
        EDYN_ASSERT(i < 2);
        return (&x)[i];
    }
};

// Zero vector.
inline constexpr vector2 vector2_zero {0, 0};

// Vector with all elements set to 1.
inline constexpr vector2 vector2_one {1, 1};

// Unit vector pointing in the x direction.
inline constexpr vector2 vector2_x {1, 0};

// Unit vector pointing in the y direction.
inline constexpr vector2 vector2_y {0, 1};

// Vector with minumum values.
inline constexpr vector2 vector2_min {EDYN_SCALAR_MIN, EDYN_SCALAR_MIN};

// Vector with maximum values.
inline constexpr vector2 vector2_max {EDYN_SCALAR_MAX, EDYN_SCALAR_MAX};

// Add two vectors.
constexpr vector2 operator+(const vector2 &v, const vector2 &w) noexcept {
    return {v.x + w.x, v.y + w.y};
}

// Add a vector into another vector.
constexpr vector2& operator+=(vector2 &v, const vector2 &w) noexcept {
    v.x += w.x;
    v.y += w.y;
    return v;
}

// Subtract two vectors.
constexpr vector2 operator-(const vector2 &v, const vector2 &w) noexcept {
    return {v.x - w.x, v.y - w.y};
}

// Subtract a vector from another vector.
constexpr vector2& operator-=(vector2 &v, const vector2 &w) noexcept {
    v.x -= w.x;
    v.y -= w.y;
    return v;
}

// Negation of a vector.
constexpr vector2 operator-(const vector2 &v) noexcept {
    return {-v.x, -v.y};
}

// Multiply vectors component-wise.
constexpr vector2 operator*(const vector2 &v, const vector2 &w) noexcept {
    return {v.x * w.x, v.y * w.y};
}

// Multiply vector by scalar.
constexpr vector2 operator*(const vector2& v, scalar s) noexcept {
    return {v.x * s, v.y * s};
}

// Multiply scalar by vector.
constexpr vector2 operator*(scalar s, const vector2 &v) noexcept {
    return {s * v.x, s * v.y};
}

// Divide vector by scalar.
constexpr vector2 operator/(const vector2 &v, scalar s) noexcept {
    return {v.x / s, v.y / s};
}

// Divide scalar by vector.
constexpr vector2 operator/(scalar s, const vector2 &v) noexcept {
    return {s / v.x, s / v.y};
}

// Scale a vector.
constexpr vector2& operator*=(vector2 &v, scalar s) noexcept {
    v.x *= s;
    v.y *= s;
    return v;
}

// Inverse-scale a vector.
constexpr vector2& operator/=(vector2 &v, scalar s) noexcept {
    auto z = scalar(1) / s;
    v.x *= z;
    v.y *= z;
    return v;
}

// Dot product between vectors.
constexpr scalar dot(const vector2 &v, const vector2 &w) noexcept {
    return v.x * w.x + v.y * w.y;
}

// Dot product between a vector and a perpendicular to the other vector.
constexpr scalar perp_product(const vector2 &v, const vector2 &w) noexcept {
    return v.x * w.y - v.y * w.x;
}

// Vector orthogonal to argument, i.e. rotated 90 degrees counter-clockwise.
// Negate result for a clockwise 90 degree rotation.
constexpr vector2 orthogonal(const vector2 &v) noexcept {
    return {-v.y, v.x};
}

// Square length of a vector.
constexpr scalar length_sqr(const vector2 &v) noexcept {
    return dot(v, v);
}

// Length of a vector.
inline scalar length(const vector2 &v) noexcept {
    return std::sqrt(length_sqr(v));
}

// Distance between two points.
inline scalar distance(const vector2 &p0, const vector2 &p1) noexcept {
    return length(p0 - p1);
}

// Squared distance between two points.
constexpr scalar distance_sqr(const vector2 &p0, const vector2 &p1) noexcept {
    return length_sqr(p0 - p1);
}

// Normalized vector (unit length). Asserts if the vector's length is zero.
inline vector2 normalize(const vector2 &v) noexcept {
    auto l = length(v);
    EDYN_ASSERT(l > EDYN_EPSILON);
    return v / l;
}

}

#endif // EDYN_MATH_VECTOR2_HPP
