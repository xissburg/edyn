#ifndef EDYN_SERIALIZATION_MATH_S11N_HPP
#define EDYN_SERIALIZATION_MATH_S11N_HPP

#include "edyn/math/vector3.hpp"

namespace edyn {

template<typename Archive>
void serialize(Archive &archive, vector3 &v) {
    archive(v.x, v.y, v.z);
}

}

#endif // EDYN_SERIALIZATION_MATH_S11N_HPP