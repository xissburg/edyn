#ifndef EDYN_MATH_QUATERNION_HPP
#define EDYN_MATH_QUATERNION_HPP

#include "vector3.hpp"
#include "constants.hpp"

namespace edyn {

struct quaternion {
    scalar x, y, z, w;

    scalar& operator[](size_t i) noexcept {
        EDYN_ASSERT(i < 4);
        return (&x)[i];
    }

    scalar operator[](size_t i) const noexcept {
        EDYN_ASSERT(i < 4);
        return (&x)[i];
    }
};

inline constexpr quaternion quaternion_identity {0, 0, 0, 1};

// Add two quaternions.
constexpr quaternion operator+(const quaternion& q0, const quaternion &q1) noexcept {
    return {q0.x + q1.x, q0.y + q1.y, q0.z + q1.z, q0.w + q1.w};
}

// Add a quaternion into another quaternion.
constexpr quaternion& operator+=(quaternion &q0, const quaternion &q1) noexcept {
    q0.x += q1.x;
    q0.y += q1.y;
    q0.z += q1.z;
    q0.w += q1.w;
    return q0;
}

// Subtract two quaternions.
constexpr quaternion operator-(const quaternion &q0, const quaternion &q1) noexcept {
    return {q0.x - q1.x, q0.y - q1.y, q0.z - q1.z, q0.w - q1.w};
}

// Subtract a quaternion from another quaternion.
constexpr quaternion& operator-=(quaternion &q0, const quaternion &q1) noexcept {
    q0.x -= q1.x;
    q0.y -= q1.y;
    q0.z -= q1.z;
    q0.w -= q1.w;
    return q0;
}

// Multiply quaternion by scalar.
constexpr quaternion operator*(const quaternion& q, scalar s) noexcept {
    return {q.x * s, q.y * s, q.z * s, q.w * s};
}

// Multiply scalar by quaternion.
constexpr quaternion operator*(scalar s, const quaternion &q) noexcept {
    return {s * q.x, s * q.y, s * q.z, s * q.w};
}

// Divide quaternion by scalar.
constexpr quaternion operator/(const quaternion &q, scalar s) noexcept {
    return {q.x / s, q.y / s, q.z / s, q.w / s};
}

// Divide scalar by quaternion.
constexpr quaternion operator/(scalar s, const quaternion &q) noexcept {
    return {s / q.x, s / q.y, s / q.z, s / q.w};
}

// Product of two quaternions.
constexpr quaternion operator*(const quaternion &q, const quaternion &r) noexcept {
    return {
        q.w * r.x + q.x * r.w + q.y * r.z - q.z * r.y,
        q.w * r.y + q.y * r.w + q.z * r.x - q.x * r.z,
        q.w * r.z + q.z * r.w + q.x * r.y - q.y * r.x,
        q.w * r.w - q.x * r.x - q.y * r.y - q.z * r.z
    };
}

constexpr quaternion& operator*=(quaternion &q, const quaternion &r) noexcept {
    return q = q * r;
}

// Product of a quaternion and vector, i.e. product of a quaternion with another
// quaternion with a zero w component.
constexpr quaternion operator*(const quaternion &q, const vector3 &v) noexcept {
    return {
        q.w * v.x + q.y * v.z - q.z * v.y,
        q.w * v.y + q.z * v.x - q.x * v.z,
        q.w * v.z + q.x * v.y - q.y * v.x,
       -q.x * v.x - q.y * v.y - q.z * v.z
    };
}

// Product of a vector and a quaternion.
constexpr quaternion operator*(const vector3 &v, const quaternion &q) noexcept {
    return {
        v.x * q.w + v.y * q.z - v.z * q.y,
        v.y * q.w + v.z * q.x - v.x * q.z,
        v.z * q.w + v.x * q.y - v.y * q.x,
       -v.x * q.x - v.y * q.y - v.z * q.z
    };
}

// Check if two quaternions are equal.
constexpr bool operator==(const quaternion &q, const quaternion &v) noexcept {
    return q.x == v.x && q.y == v.y && q.z == v.z && q.w == v.w;
}

// Check if two quaternions are different.
constexpr bool operator!=(const quaternion &q, const quaternion &v) noexcept {
    return q.x != v.x || q.y != v.y || q.z != v.z || q.w != v.w;
}

// Squared length of a quaternion.
constexpr scalar length_sqr(const quaternion &q) noexcept {
    return q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
}

// Length of a quaternion.
inline scalar length(const quaternion &q) noexcept {
    return std::sqrt(length_sqr(q));
}

constexpr scalar dot(const quaternion &q0, const quaternion &q1) noexcept {
    return q0.x * q1.x + q0.y * q1.y + q0.z * q1.z + q0.w * q1.w;
}

// Returns a unit-length version of the given quaternion.
inline quaternion normalize(const quaternion &q) noexcept {
    auto l = length(q);
    EDYN_ASSERT(l > EDYN_EPSILON);
    return q / l;
}

// Conjugate of a quaternion.
constexpr quaternion conjugate(const quaternion &q) noexcept {
    return {-q.x, -q.y, -q.z, q.w};
}

// Rotate a vector by a quaternion.
constexpr vector3 rotate(const quaternion &q, const vector3 &v) noexcept {
    // Formula from https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#Performance_comparisons
    auto r = vector3{q.x, q.y, q.z};
    return v + cross(scalar(2) * r, cross(r, v) + q.w * v);
}

/**
 * @brief Make a quaternion representing a rotation about an axis.
 * @param v Rotation axis. Doesn't have to be unit length.
 * @param a Angle in radians.
 * @return Quaternion that rotates by `a` about axis `v`.
 */
inline quaternion quaternion_axis_angle(const vector3 &v, scalar a) noexcept {
    auto l = length(v);
    auto s = std::sin(a * scalar(0.5)) / l;
    return {v.x * s, v.y * s, v.z * s, std::cos(a * scalar(0.5))};
}

/**
 * @brief Make a quaternion representing a rotation about an unit axis.
 * @param v Rotation axis assumed to be unit length.
 * @param a Angle in radians.
 * @return Quaternion that rotates by `a` about axis `v`.
 */
inline quaternion quaternion_axis_angle_unit(const vector3 &v, scalar a) noexcept {
    EDYN_ASSERT(std::abs(length_sqr(v) - 1) < EDYN_EPSILON);
    auto s = std::sin(a * scalar(0.5));
    return {v.x * s, v.y * s, v.z * s, std::cos(a * scalar(0.5))};
}

// Get rotation angle of a quaternion.
inline scalar quaternion_angle(const quaternion &q) noexcept {
    return std::acos(q.w) * scalar(2);
}

// Get rotation axis of a quaternion.
inline vector3 quaternion_axis(const quaternion &q) noexcept {
    auto s2 = scalar(1) - q.w * q.w;

    if (s2 > EDYN_EPSILON) {
        auto s = scalar(1) / std::sqrt(s2);
        return {q.x * s, q.y * s, q.z * s};
    }

    return vector3_x;
}

// Get x-axis of the basis of a quaternion.
inline vector3 quaternion_x(const quaternion &q) noexcept {
    return rotate(q, vector3_x);
}

// Get y-axis of the basis of a quaternion.
inline vector3 quaternion_y(const quaternion &q) noexcept {
    return rotate(q, vector3_y);
}

// Get z-axis of the basis of a quaternion.
inline vector3 quaternion_z(const quaternion &q) noexcept {
    return rotate(q, vector3_z);
}

// Spherical linear interpolation.
inline quaternion slerp(const quaternion &q0, const quaternion &q1, scalar s) noexcept {
    const auto magnitude = std::sqrt(length_sqr(q0) * length_sqr(q1));
    EDYN_ASSERT(magnitude > 0);

    const auto prod = dot(q0, q1) / magnitude;
    const auto abs_prod = std::abs(prod);

    if (abs_prod > scalar(1) - EDYN_EPSILON) {
        return q0;
    }

    const auto theta = std::acos(abs_prod);
    const auto d = std::sin(theta);
    EDYN_ASSERT(d > 0);

    const auto sign = prod < 0 ? scalar(-1) : scalar(1);
    const auto s0 = std::sin((scalar(1) - s) * theta) / d;
    const auto s1 = std::sin(sign * s * theta) / d;

    return {
        q0.x * s0 + q1.x * s1,
        q0.y * s0 + q1.y * s1,
        q0.z * s0 + q1.z * s1,
        q0.w * s0 + q1.w * s1
    };
}

// Integrate angular velocity over time.
quaternion integrate(const quaternion &q, const vector3 &w, scalar dt) noexcept;

// Returns the shortest rotation that takes `v0` to `v1`.
quaternion shortest_arc(const vector3 &v0, const vector3 &v1) noexcept;

// Returns the angle between two quaternions along the shortest path.
scalar angle_between(const quaternion &q0, const quaternion &q1) noexcept;

// Derivative of a quaternion along an axis-angle.
// Reference: https://fgiesen.wordpress.com/2012/08/24/quaternion-differentiation/
constexpr quaternion quaternion_derivative(const quaternion &q, const vector3 &w) noexcept {
    return quaternion{w.x, w.y, w.z, 0} * q * scalar(0.5);
}

}

#endif // EDYN_MATH_QUATERNION_HPP
