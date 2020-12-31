#ifndef EDYN_SERIALIZATION_COMP_LINVEL_S11N_HPP
#define EDYN_SERIALIZATION_COMP_LINVEL_S11N_HPP

#include "edyn/comp/linvel.hpp"

namespace edyn {

template<typename Archive>
void serialize(Archive &archive, linvel &v) {
    archive(v.x, v.y, v.z);
}

}

#endif // EDYN_SERIALIZATION_COMP_LINVEL_S11N_HPP