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

template<typename Archive>
void serialize(Archive &archive, present_orientation &v) {
    archive(v.x, v.y, v.z, v.w);
}

}

#endif // EDYN_COMP_PRESENT_ORIENTATION_HPP