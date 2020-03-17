#ifndef EDYN_COMP_MATERIAL_HPP
#define EDYN_COMP_MATERIAL_HPP

#include "edyn/math/scalar.hpp"
#include "edyn/math/constants.hpp"

namespace edyn {

struct material {
    scalar restitution {0};
    scalar friction {0.5};
    scalar stiffness {large_scalar};
    scalar damping {large_scalar};
};

}

#endif // EDYN_COMP_MATERIAL_HPP