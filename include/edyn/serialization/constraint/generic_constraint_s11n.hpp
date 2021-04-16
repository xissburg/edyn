#ifndef EDYN_SERIALIZATION_CONSTRAINT_GENERIC_CONSTRAINT_S11N_HPP
#define EDYN_SERIALIZATION_CONSTRAINT_GENERIC_CONSTRAINT_S11N_HPP

#include "edyn/constraints/generic_constraint.hpp"

namespace edyn {

template<typename Archive>
void serialize(Archive &archive, generic_constraint &c) {
    archive(c.body);
    archive(c.pivot);
}

}

#endif // EDYN_SERIALIZATION_CONSTRAINT_GENERIC_CONSTRAINT_S11N_HPP