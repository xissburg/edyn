#ifndef EDYN_COMP_ROLL_DIRECTION_HPP
#define EDYN_COMP_ROLL_DIRECTION_HPP

#include "edyn/math/vector3.hpp"

namespace edyn {

/**
 * @brief Assign to shapes that can roll in only one direction, such as
 * cylinders. It stores a unit vector in object space which is the
 * rotation axis.
 */
struct roll_direction : public vector3 {};

}

#endif // EDYN_COMP_ROLL_DIRECTION_HPP
