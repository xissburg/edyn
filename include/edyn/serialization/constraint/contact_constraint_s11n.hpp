#ifndef EDYN_SERIALIZATION_CONSTRAINT_CONTACT_CONSTRAINT_S11N_HPP
#define EDYN_SERIALIZATION_CONSTRAINT_CONTACT_CONSTRAINT_S11N_HPP

#include "edyn/constraints/contact_constraint.hpp"

namespace edyn {

template<typename Archive>
void serialize(Archive &archive, contact_constraint &c) {
    archive(c.body);
}

}

#endif // EDYN_SERIALIZATION_CONSTRAINT_CONTACT_CONSTRAINT_S11N_HPP