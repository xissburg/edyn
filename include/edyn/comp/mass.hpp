#ifndef EDYN_COMP_MASS_HPP
#define EDYN_COMP_MASS_HPP

#include "scalar_comp.hpp"

namespace edyn {

struct mass : public scalar_comp {};

struct mass_inv : public scalar_comp {};

}

#endif // EDYN_COMP_MASS_HPP