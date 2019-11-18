#ifndef EDYN_COMP_DELTA_ANGVEL_HPP
#define EDYN_COMP_DELTA_ANGVEL_HPP

#include "edyn/math/vector3.hpp"

namespace edyn {

struct delta_angvel : public vector3 {
    delta_angvel& operator=(const vector3& v) {
        vector3::operator=(v);
        return *this;
    }
};

}


#endif // EDYN_COMP_DELTA_ANGVEL_HPP