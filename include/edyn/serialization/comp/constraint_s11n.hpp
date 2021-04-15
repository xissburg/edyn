#ifndef EDYN_SERIALIZATION_COMP_CONSTRAINT_S11N_HPP
#define EDYN_SERIALIZATION_COMP_CONSTRAINT_S11N_HPP

#include "edyn/comp/constraint.hpp"

namespace edyn {

template<typename Archive>
void serialize(Archive &archive, constraint &c) {
    archive(c.body);
    archive(c.var);
    archive(c.impulse);
}

}

#endif // EDYN_SERIALIZATION_COMP_CONSTRAINT_S11N_HPP