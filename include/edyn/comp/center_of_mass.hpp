#ifndef EDYN_COMP_CENTER_OF_MASS_HPP
#define EDYN_COMP_CENTER_OF_MASS_HPP

#include "edyn/math/vector3.hpp"

namespace edyn {

struct center_of_mass : public vector3 {
    center_of_mass & operator=(const vector3 &v) {
        vector3::operator=(v);
        return *this;
    }
};

}

#endif // EDYN_COMP_CENTER_OF_MASS_HPP
