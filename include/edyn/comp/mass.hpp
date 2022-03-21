#ifndef EDYN_COMP_MASS_HPP
#define EDYN_COMP_MASS_HPP

#include "scalar_comp.hpp"

namespace edyn {

struct mass : public scalar_comp {};

struct mass_inv : public scalar_comp {};

template<typename Archive>
void serialize(Archive &archive, mass &m) {
    archive(m.s);
}

template<typename Archive>
void serialize(Archive &archive, mass_inv &m) {
    archive(m.s);
}

}

#endif // EDYN_COMP_MASS_HPP
