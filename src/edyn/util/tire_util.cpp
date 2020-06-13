#include "edyn/util/tire_util.hpp"
#include "edyn/math/constants.hpp"

namespace edyn {

scalar velocity_dependent_vertical_stiffness(scalar nominal_stiffness, scalar speed, scalar inflation_pressure) {
    scalar pinfl = inflation_pressure * 1.1;
    return nominal_stiffness * (pinfl/inflation_pressure) * (1 + 2.4e-3*(speed - 16.67));
}

}