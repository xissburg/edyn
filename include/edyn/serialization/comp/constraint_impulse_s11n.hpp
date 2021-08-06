#ifndef EDYN_SERIALIZATION_COMP_CONSTRAINT_IMPULSE_S11N_HPP
#define EDYN_SERIALIZATION_COMP_CONSTRAINT_IMPULSE_S11N_HPP

#include "edyn/constraints/constraint_impulse.hpp"

namespace edyn {

template<typename Archive>
void serialize(Archive &archive, constraint_impulse &imp) {
    archive(imp.values);
}

}

#endif // EDYN_SERIALIZATION_COMP_CONSTRAINT_IMPULSE_S11N_HPP
