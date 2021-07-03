#ifndef EDYN_COMP_SPIN_HPP
#define EDYN_COMP_SPIN_HPP

#include "scalar_comp.hpp"

namespace edyn {

struct spin : public scalar_comp {};

struct spin_angle : public scalar_comp {
    long count {0}; // Number of complete spins (2π rad).
};

struct present_spin_angle : public scalar_comp {};

struct delta_spin : public scalar_comp {};

}

#endif // EDYN_COMP_SPIN_HPP
