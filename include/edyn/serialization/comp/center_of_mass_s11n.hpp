#ifndef EDYN_SERIALIZATION_COMP_CENTER_OF_MASS_S11N_HPP
#define EDYN_SERIALIZATION_COMP_CENTER_OF_MASS_S11N_HPP

#include "edyn/comp/center_of_mass.hpp"

namespace edyn {

template<typename Archive>
void serialize(Archive &archive, center_of_mass &com) {
    archive(com.x, com.y, com.z);
}

}

#endif // EDYN_SERIALIZATION_COMP_CENTER_OF_MASS_S11N_HPP
