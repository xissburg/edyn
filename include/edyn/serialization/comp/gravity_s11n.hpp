#ifndef EDYN_SERIALIZATION_COMP_GRAVITY_S11N_HPP
#define EDYN_SERIALIZATION_COMP_GRAVITY_S11N_HPP

#include "edyn/comp/gravity.hpp"

namespace edyn {

template<typename Archive>
void serialize(Archive &, gravity &) {}

}

#endif // EDYN_SERIALIZATION_COMP_GRAVITY_S11N_HPP