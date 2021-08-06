#ifndef EDYN_SERIALIZATION_COMP_GRAVITY_S11N_HPP
#define EDYN_SERIALIZATION_COMP_GRAVITY_S11N_HPP

#include "edyn/comp/gravity.hpp"

namespace edyn {

template<typename Archive>
void serialize(Archive &archive, gravity &g) {
    archive(g.x, g.y, g.z);
}

}

#endif // EDYN_SERIALIZATION_COMP_GRAVITY_S11N_HPP
