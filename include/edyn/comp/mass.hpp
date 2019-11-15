#ifndef EDYN_COMP_MASS_HPP
#define EDYN_COMP_MASS_HPP

#include "edyn/math/scalar.hpp"

namespace edyn {

struct mass {
    scalar s;

    operator scalar&() {
        return s;
    }

    operator scalar() const {
        return s;
    }
};

}

#endif // EDYN_COMP_MASS_HPP