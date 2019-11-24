#ifndef EDYN_COMP_MATTER_HPP
#define EDYN_COMP_MATTER_HPP

#include "edyn/math/scalar.hpp"

namespace edyn {

struct matter {
    scalar restitution;
    scalar friction;
};

}

#endif // EDYN_COMP_MATTER_HPP