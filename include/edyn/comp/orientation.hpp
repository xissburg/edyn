#ifndef EDYN_COMP_ORIENTATION_HPP
#define EDYN_COMP_ORIENTATION_HPP

#include "edyn/math/quaternion.hpp"

namespace edyn {

struct orientation : public quaternion {
    orientation & operator=(const quaternion &q) {
        quaternion::operator=(q);
        return *this;
    }
};

}


#endif // EDYN_COMP_ORIENTATION_HPP