#ifndef EDYN_UTIL_TIRE_HPP
#define EDYN_UTIL_TIRE_HPP

#include <cstdint>
#include "edyn/math/scalar.hpp"

namespace edyn {

scalar velocity_dependent_vertical_stiffness(scalar nominal_stiffness, 
                                             scalar speed, 
                                             scalar inflation_pressure = 200000);

}

#endif // EDYN_UTIL_TIRE_HPP