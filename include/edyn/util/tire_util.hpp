#ifndef EDYN_UTIL_TIRE_UTIL_HPP
#define EDYN_UTIL_TIRE_UTIL_HPP

#include "edyn/math/math.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/math/quaternion.hpp"

namespace edyn {

scalar velocity_dependent_vertical_stiffness(scalar nominal_stiffness,
                                             scalar speed_kph,
                                             scalar inflation_pressure = 200000);

}

#endif // EDYN_UTIL_TIRE_UTIL_HPP
