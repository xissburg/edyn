#ifndef EDYN_SERIALIZATION_COMP_MATERIAL_S11N_HPP
#define EDYN_SERIALIZATION_COMP_MATERIAL_S11N_HPP

#include "edyn/comp/material.hpp"

namespace edyn {

template<typename Archive>
void serialize(Archive &archive, material &m) {
    archive(m.restitution);
    archive(m.friction);
    archive(m.stiffness);
    archive(m.damping);
}

}

#endif // EDYN_SERIALIZATION_COMP_MATERIAL_S11N_HPP