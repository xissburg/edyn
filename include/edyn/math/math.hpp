#ifndef EDYN_MATH_MATH_HPP
#define EDYN_MATH_MATH_HPP

#include "constants.hpp"
#include <algorithm>
#include <cmath>
#include <array>

namespace edyn {

/**
 * @return Value of `radians` converted to degrees.
 */
inline scalar to_degrees(scalar radians) {
    return radians / pi * 180;
}

/**
 * @return Value of `degress` converted to radians.
 */
inline scalar to_radians(scalar degrees) {
    return degrees / 180 * pi;
}

/**
 * Converts torque units in Nm/degree to Nm/rad.
 * @param Nm_per_degree Nm/degree.
 * @return Torque in Nm/rad.
 */
inline scalar to_Nm_per_radian(scalar Nm_per_degree) {
    return Nm_per_degree * to_degrees(1);
}

/**
 * Converts torque units in Nm/rad to Nm/degree.
 * @param Nm_per_radian Nm/rad.
 * @return Torque in Nm/degree.
 */
inline scalar to_Nm_per_degree(scalar Nm_per_radian) {
    return Nm_per_radian / to_degrees(1);
}

/**
 * @return Scalar clamped to the [0, 1] interval.
 */
inline scalar clamp_unit(scalar s) {
    return std::clamp(s, scalar(0), scalar(1));
}

/**
 * @return Angle in [-π, π].
 */
inline scalar normalize_angle(scalar s) {
    s = std::fmod(s, pi2);

    if (s < -pi) {
        return s + pi2;
    } else if (s > pi) {
        return s - pi2;
    }

    return s;
}

/**
 * @return Linear interpolation between `a` and `b` by scalar `s`.
 */
template<typename T, typename Scalar>
inline auto lerp(T a, T b, Scalar s) {
    return a * (Scalar(1) - s) + b * s;
}

/**
 * @return The square of a number.
 */
template<typename T>
inline auto square(T a) {
    return a * a;
}

/**
 * @return 1 if `b` is true, -1 if `b is false.
 */
inline auto to_sign(bool b) {
    return b ? scalar(1) : scalar(-1);
}

/**
 * @brief Calculate average of N values, i.e. their sum divided by N.
 * @param array Array of values.
 * @return Average value.
 */
template<typename T, size_t N>
inline auto average(const std::array<T, N> &array) {
    auto sum = array[0];
    for (size_t i = 1; i < N; ++i) {
        sum += array[i];
    }
    return sum / N;
}

}

#endif // EDYN_MATH_MATH_HPP
