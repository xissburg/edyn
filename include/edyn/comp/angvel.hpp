#ifndef EDYN_COMP_ANGVEL_HPP
#define EDYN_COMP_ANGVEL_HPP

#include "edyn/math/vector3.hpp"

namespace edyn {
/**
 * @brief Angular velocity component.
 */
struct angvel : public vector3 {
    angvel & operator=(const vector3 &v) {
        vector3::operator=(v);
        return *this;
    }
};

}

#endif // EDYN_COMP_ANGVEL_HPP