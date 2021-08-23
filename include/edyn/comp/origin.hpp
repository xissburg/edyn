#ifndef EDYN_COMP_ORIGIN_HPP
#define EDYN_COMP_ORIGIN_HPP

#include "edyn/math/vector3.hpp"

namespace edyn {

/**
 * @brief Origin of a rigid body in world space. This is present in dynamic
 * rigid bodies which have a non-zero center of mass offset.
 */
struct origin : public vector3 {
    origin & operator=(const vector3 &v) {
        vector3::operator=(v);
        return *this;
    }
};

}

#endif // EDYN_COMP_ORIGIN_HPP
