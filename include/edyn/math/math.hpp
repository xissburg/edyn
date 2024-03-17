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
constexpr scalar to_degrees(scalar radians) noexcept {
    return radians / pi * 180;
}

/**
 * @return Value of `degrees` converted to radians.
 */
constexpr scalar to_radians(scalar degrees) noexcept {
    return degrees / 180 * pi;
}

/**
 * Converts torque units in Nm/degree to Nm/rad.
 * @param Nm_per_degree Nm/degree.
 * @return Torque in Nm/rad.
 */
constexpr scalar to_Nm_per_radian(scalar Nm_per_degree) noexcept {
    return Nm_per_degree * to_degrees(1);
}

/**
 * Converts torque units in Nm/rad to Nm/degree.
 * @param Nm_per_radian Nm/rad.
 * @return Torque in Nm/degree.
 */
constexpr scalar to_Nm_per_degree(scalar Nm_per_radian) noexcept {
    return Nm_per_radian / to_degrees(1);
}

/**
 * @return Scalar clamped to the [0, 1] interval.
 */
constexpr scalar clamp_unit(scalar s) noexcept {
    return std::clamp(s, scalar(0), scalar(1));
}

/**
 * @return Angle in [-π, π].
 */
inline scalar normalize_angle(scalar s) noexcept {
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
constexpr auto lerp(T a, T b, Scalar s) noexcept {
    return a * (Scalar(1) - s) + b * s;
}

/**
 * @return The square of a number.
 */
template<typename T>
constexpr auto square(T a) noexcept {
    return a * a;
}

/**
 * @return 1 if `b` is true, -1 if `b is false.
 */
constexpr auto to_sign(bool b) noexcept {
    return b ? scalar(1) : scalar(-1);
}

/**
 * @brief Calculate average of N values, i.e. their sum divided by N.
 * @param array Array of values.
 * @return Average value.
 */
template<typename T, size_t N>
constexpr auto average(const std::array<T, N> &array) noexcept {
    auto sum = array[0];
    for (size_t i = 1; i < N; ++i) {
        sum += array[i];
    }
    return sum / N;
}

}

#endif // EDYN_MATH_MATH_HPP
