#ifndef EDYN_COMP_PRESENT_ORIENTATION_HPP
#define EDYN_COMP_PRESENT_ORIENTATION_HPP

#include "edyn/math/quaternion.hpp"

namespace edyn {

struct present_orientation : public quaternion {
    present_orientation & operator=(const quaternion &q) {
        quaternion::operator=(q);
        return *this;
    }
};

}

#endif // EDYN_COMP_PRESENT_ORIENTATION_HPP
