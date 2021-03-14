#ifndef EDYN_SERIALIZATION_COMP_ANGVEL_S11N_HPP
#define EDYN_SERIALIZATION_COMP_ANGVEL_S11N_HPP

#include "edyn/comp/angvel.hpp"

namespace edyn {

template<typename Archive>
void serialize(Archive &archive, angvel &v) {
    archive(v.x, v.y, v.z);
}

}

#endif // EDYN_SERIALIZATION_COMP_ANGVEL_S11N_HPP