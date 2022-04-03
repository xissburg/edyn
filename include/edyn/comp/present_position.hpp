#ifndef EDYN_COMP_PRESENT_POSITION_HPP
#define EDYN_COMP_PRESENT_POSITION_HPP

#include "edyn/math/vector3.hpp"

namespace edyn {

struct present_position : public vector3 {
    present_position & operator=(const vector3 &v) {
        vector3::operator=(v);
        return *this;
    }
};

}

#endif // EDYN_COMP_PRESENT_POSITION_HPP
