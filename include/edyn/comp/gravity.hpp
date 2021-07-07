#ifndef EDYN_COMP_GRAVITY_HPP
#define EDYN_COMP_GRAVITY_HPP

#include "edyn/math/vector3.hpp"

namespace edyn {

struct gravity : public vector3 {
    gravity & operator=(const vector3 &v) {
        vector3::operator=(v);
        return *this;
    }
};

}

#endif // EDYN_COMP_GRAVITY_HPP