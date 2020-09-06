#ifndef EDYN_SERIALIZATION_COMP_MASS_S11N_HPP
#define EDYN_SERIALIZATION_COMP_MASS_S11N_HPP

#include "edyn/comp/mass.hpp"

namespace edyn {

template<typename Archive>
void serialize(Archive &archive, mass &m) {
    archive(m.s);
}

template<typename Archive>
void serialize(Archive &archive, mass_inv &m) {
    archive(m.s);
}

}

#endif // EDYN_SERIALIZATION_COMP_MASS_S11N_HPP