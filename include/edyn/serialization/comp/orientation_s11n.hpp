#ifndef EDYN_SERIALIZATION_COMP_ORIENTATION_S11N_HPP
#define EDYN_SERIALIZATION_COMP_ORIENTATION_S11N_HPP

#include "edyn/comp/orientation.hpp"

namespace edyn {

template<typename Archive>
void serialize(Archive &archive, orientation &v) {
    archive(v.x, v.y, v.z, v.w);
}

}

#endif // EDYN_SERIALIZATION_COMP_ORIENTATION_S11N_HPP