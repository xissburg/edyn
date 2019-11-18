#ifndef EDYN_COMP_DELTA_LINVEL_HPP
#define EDYN_COMP_DELTA_LINVEL_HPP

#include "edyn/math/vector3.hpp"

namespace edyn {

struct delta_linvel : public vector3 {
    delta_linvel& operator=(const vector3& v) {
        vector3::operator=(v);
        return *this;
    }
};

}

#endif // EDYN_COMP_DELTA_LINVEL_HPP