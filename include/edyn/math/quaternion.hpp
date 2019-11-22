#ifndef EDYN_MATH_QUATERNION_HPP
#define EDYN_MATH_QUATERNION_HPP

#include "vector3.hpp"

namespace edyn {

struct quaternion {
    scalar x, y, z, w;
};

inline constexpr quaternion quaternion_identity {0, 0, 0, 1};

// Multiply quaternion by scalar.
inline quaternion operator*(const quaternion& q, scalar s) {
    return {q.x * s, q.y * s, q.z * s, q.w * s};
}

// Multiply scalar by quaternion.
inline quaternion operator*(scalar s, const quaternion &q) {
    return {s * q.x, s * q.y, s * q.z, s * q.w};
}

// Divide quaternion by scalar.
inline quaternion operator/(const quaternion &q, scalar s) {
    return {q.x / s, q.y / s, q.z / s, q.w / s};
}

// Divide scalar by quaternion.
inline quaternion operator/(scalar s, const quaternion &q) {
    return {s / q.x, s / q.y, s / q.z, s / q.w};
}

// Product of two quaternions.
inline quaternion operator*(const quaternion &q, const quaternion &r) {
    return {
        q.w * r.x + q.x * r.w + q.y * r.z - q.z * r.y,
        q.w * r.y + q.y * r.w + q.z * r.x - q.x * r.z,
        q.w * r.z + q.z * r.w + q.x * r.y - q.y * r.x,
        q.w * r.w - q.x * r.x - q.y * r.y - q.z * r.z
    };
}

// Product of a quaternion and vector, i.e. product of a quaternion with another
// quaternion with a zero w component.
inline quaternion operator*(const quaternion &q, const vector3 &v) {
    return {
        q.w * v.x + q.y * v.z - q.z * v.y,
        q.w * v.y + q.z * v.x - q.x * v.z,
        q.w * v.z + q.x * v.y - q.y * v.x,
       -q.x * v.x - q.y * v.y - q.z * v.z
    };
}

// Product of a vector and a quaternion.
inline quaternion operator*(const vector3 &v, const quaternion &q) {
    return {
        v.x * q.w + v.y * q.z - v.z * q.y,
        v.y * q.w + v.z * q.x - v.x * q.z,
        v.z * q.w + v.x * q.y - v.y * q.x,
       -v.x * q.x - v.y * q.y - v.z * q.z
    };
}

// Squared length of a quaternion.
inline scalar length2(const quaternion &q) {
    return q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
}

// Length of a quaternion.
inline scalar length(const quaternion &q) {
    return std::sqrt(length2(q));
}

inline quaternion normalize(const quaternion &q) {
    auto l = length(q);
    EDYN_ASSERT(l > EDYN_EPSILON);
    return q / l;
}

// Inverse of a quaternion.
inline quaternion inverse(const quaternion &q) {
    return {-q.x, -q.y, -q.z, q.w};
}

// Rotate a vector by a quaternion.
inline vector3 rotate(const quaternion &q, const vector3 &v) {
    auto r = q * v * inverse(q);
    return {r.x, r.y, r.z};
}

// Integrate angular velocity over time.
inline quaternion integrate(const quaternion &q, const vector3 &w, scalar dt) {
    // "Practical Parameterization of Rotations Using the Exponential Map", F. Sebastian Grassia
    const auto ws = length(w);
    const auto min_ws = scalar {0.001};
    scalar half = 0.5;
    scalar t;

    if (ws < min_ws) {
        constexpr auto k = scalar(1) / scalar(48);
        t = half * dt - dt * dt * dt * k * ws * ws;
    } else {
        t = std::sin(half * ws * dt) / ws;
    }

    auto r = quaternion {w.x * t, w.y * t, w.z * t, std::cos(half * ws * dt)};
    return normalize(r * q);
}

}

#endif // EDYN_MATH_QUATERNION_HPP