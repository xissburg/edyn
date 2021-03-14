#ifndef EDYN_MATH_VECTOR2_HPP
#define EDYN_MATH_VECTOR2_HPP

#include <cmath>
#include "scalar.hpp"
#include "edyn/config/config.h"

namespace edyn {

struct vector2 {
    scalar x, y;

    scalar& operator[](size_t i) {
        EDYN_ASSERT(i < 2);
        return (&x)[i];
    }

    scalar operator[](size_t i) const {
        EDYN_ASSERT(i < 2);
        return (&x)[i];
    }
};

// Add two vectors.
inline vector2 operator+(const vector2 &v, const vector2 &w) {
    return {v.x + w.x, v.y + w.y};
}

// Add a vector into another vector.
inline vector2& operator+=(vector2 &v, const vector2 &w) {
    v.x += w.x;
    v.y += w.y;
    return v;
}

// Subtract two vectors.
inline vector2 operator-(const vector2 &v, const vector2 &w) {
    return {v.x - w.x, v.y - w.y};
}

// Subtract a vector from another vector.
inline vector2& operator-=(vector2 &v, const vector2 &w) {
    v.x -= w.x;
    v.y -= w.y;
    return v;
}

// Negation of a vector.
inline vector2 operator-(const vector2 &v) {
    return {-v.x, -v.y};
}

// Multiply vectors component-wise.
inline vector2 operator*(const vector2 &v, const vector2 &w) {
    return {v.x * w.x, v.y * w.y};
}

// Multiply vector by scalar.
inline vector2 operator*(const vector2& v, scalar s) {
    return {v.x * s, v.y * s};
}

// Multiply scalar by vector.
inline vector2 operator*(scalar s, const vector2 &v) {
    return {s * v.x, s * v.y};
}

// Divide vector by scalar.
inline vector2 operator/(const vector2 &v, scalar s) {
    return {v.x / s, v.y / s};
}

// Divide scalar by vector.
inline vector2 operator/(scalar s, const vector2 &v) {
    return {s / v.x, s / v.y};
}

// Scale a vector.
inline vector2& operator*=(vector2 &v, scalar s) {
    v.x *= s;
    v.y *= s;
    return v;
}

// Inverse-scale a vector.
inline vector2& operator/=(vector2 &v, scalar s) {
    auto z = scalar(1) / s;
    v.x *= z;
    v.y *= z;
    return v;
}

// Dot product between vectors.
inline scalar dot(const vector2 &v, const vector2 &w) {
    return v.x * w.x + v.y * w.y;
}

// Dot product between a vector and a perpendicular to the other vector.
inline scalar perp_product(const vector2 &v, const vector2 &w) {
    return v.x * w.y - v.y * w.x;
}

// Vector perpendicular to argument, i.e. rotated 90 degrees counter-clockwise.
inline vector2 perpendicular(const vector2 &v) {
    return {-v.y, v.x};
}

// Square length of a vector.
inline scalar length_sqr(const vector2 &v) {
    return dot(v, v);
}

// Length of a vector.
inline scalar length(const vector2 &v) {
    return std::sqrt(length_sqr(v));
}

// Distance between two points.
inline scalar distance(const vector2 &p0, const vector2 &p1) {
    return length(p0 - p1);
}

// Squared distance between two points.
inline scalar distance_sqr(const vector2 &p0, const vector2 &p1) {
    return length_sqr(p0 - p1);
}

// Normalized vector (unit length). Asserts if the vector's length is zero.
inline vector2 normalize(const vector2 &v) {
    auto l = length(v);
    EDYN_ASSERT(l > EDYN_EPSILON);
    return v / l;
}

}

#endif // EDYN_MATH_VECTOR2_HPP