#ifndef EDYN_COMP_LINACC_HPP
#define EDYN_COMP_LINACC_HPP

#include "edyn/math/vector3.hpp"

namespace edyn {

struct linacc : public vector3 {
    linacc & operator=(const vector3 &v) {
        vector3::operator=(v);
        return *this;
    }
};

}

#endif // EDYN_COMP_LINACC_HPP