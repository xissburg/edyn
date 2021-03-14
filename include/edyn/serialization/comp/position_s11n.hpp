#ifndef EDYN_SERIALIZATION_COMP_POSITION_S11N_HPP
#define EDYN_SERIALIZATION_COMP_POSITION_S11N_HPP

#include "edyn/comp/position.hpp"

namespace edyn {

template<typename Archive>
void serialize(Archive &archive, position &v) {
    archive(v.x, v.y, v.z);
}

}

#endif // EDYN_SERIALIZATION_COMP_POSITION_S11N_HPP