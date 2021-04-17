#ifndef EDYN_SERIALIZATION_CONSTRAINT_HINGE_CONSTRAINT_S11N_HPP
#define EDYN_SERIALIZATION_CONSTRAINT_HINGE_CONSTRAINT_S11N_HPP

#include "edyn/constraints/hinge_constraint.hpp"

namespace edyn {

template<typename Archive>
void serialize(Archive &archive, hinge_constraint &c) {
    archive(c.body);
    archive(c.pivot);
    archive(c.frame);
}

}


#endif // EDYN_SERIALIZATION_CONSTRAINT_HINGE_CONSTRAINT_S11N_HPP