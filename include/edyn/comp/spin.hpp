#ifndef EDYN_COMP_SPIN_HPP
#define EDYN_COMP_SPIN_HPP

#include "scalar_comp.hpp"

namespace edyn {

/**
 * @brief The spin is a 7th degree of freedom which represents an additional
 * angular velocity along the x-axis.
 */
struct spin : public scalar_comp {};

/**
 * @brief The spin angle is a 7th degree of freedom which represents an additional
 * turn around the x-axis. It splits the angle into a [0, 2π) value and an integral
 * value containing the number of complete turns.
 */
struct spin_angle : public scalar_comp {
    long count {0}; // Number of complete spins (2π rad).
};

struct present_spin_angle : public scalar_comp {};

struct delta_spin : public scalar_comp {};

}

#endif // EDYN_COMP_SPIN_HPP
