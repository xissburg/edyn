#include "edyn/math/quaternion.hpp"
#include "edyn/math/geom.hpp"

namespace edyn {

// "Practical Parameterization of Rotations Using the Exponential Map", F. Sebastian Grassia
quaternion integrate(const quaternion &q, const vector3 &w, scalar dt) {
    const auto ws = length(w);
    const auto min_ws = scalar(0.001);
    constexpr auto half = scalar(0.5);
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

// Bullet Physics (btQuaternion.h), Game Programming Gems 2.10.
quaternion shortest_arc(const vector3 &v0, const vector3 &v1) {
    auto c = cross(v0, v1);
    auto d = dot(v0, v1);

    if (d <= -1 + EDYN_EPSILON) {
        vector3 n, m;
        plane_space(v0, n, m);
        return {n.x, n.y, n.z, 0};
    }

    auto s = std::sqrt((1 + d) * 2);
    auto rs = 1 / s;
    return normalize(quaternion{c.x * rs, c.y * rs, c.z * rs, s * scalar(0.5)});
}

scalar angle_between(const quaternion &q0, const quaternion &q1) {
    auto s = std::sqrt(length_sqr(q0) * length_sqr(q1));
    EDYN_ASSERT(std::abs(s) > EDYN_EPSILON);
    auto d = dot(q0, q1);
    return std::acos(d / s) * scalar(2);
}

}