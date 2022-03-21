#ifndef EDYN_COMP_VELOCITY_HPP
#define EDYN_COMP_VELOCITY_HPP

#include "edyn/math/vector3.hpp"

namespace edyn {

/**
 * @brief Linear velocity component.
 */
struct linvel : public vector3 {
    linvel & operator=(const vector3 &v) {
        vector3::operator=(v);
        return *this;
    }
};

template<typename Archive>
void serialize(Archive &archive, linvel &v) {
    archive(v.x, v.y, v.z);
}

}

#endif // EDYN_COMP_VELOCITY_HPP
