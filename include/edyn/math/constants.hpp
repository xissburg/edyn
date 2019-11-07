#ifndef EDYN_MATH_CONSTANTS_HPP
#define EDYN_MATH_CONSTANTS_HPP

#include <cmath>
#include "vector3.hpp"

namespace edyn {

inline constexpr scalar pi = 3.1415926535897932384626433832795029;

inline constexpr vector3 gravity_sun     {0, -274,   0};
inline constexpr vector3 gravity_mercury {0, -3.7,   0};
inline constexpr vector3 gravity_venus   {0, -8.87,  0};
inline constexpr vector3 gravity_earth   {0, -9.8,   0};
inline constexpr vector3 gravity_mars    {0, -3.721, 0};
inline constexpr vector3 gravity_jupiter {0, -24.79, 0};
inline constexpr vector3 gravity_saturn  {0, -10.44, 0};
inline constexpr vector3 gravity_uranus  {0, -8.69,  0};
inline constexpr vector3 gravity_neptune {0, -11.15, 0};
inline constexpr vector3 gravity_pluto   {0, -0.62,  0};
inline constexpr vector3 gravity_moon    {0, -1.625, 0};

}

#endif // EDYN_MATH_CONSTANTS_HPP