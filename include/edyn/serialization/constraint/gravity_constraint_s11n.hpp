#ifndef EDYN_SERIALIZATION_CONSTRAINT_GRAVITY_CONSTRAINT_S11N_HPP
#define EDYN_SERIALIZATION_CONSTRAINT_GRAVITY_CONSTRAINT_S11N_HPP

#include "edyn/constraints/gravity_constraint.hpp"

namespace edyn {

template<typename Archive>
void serialize(Archive &archive, gravity_constraint &con) {
    archive(con.body);
}

}

#endif // EDYN_SERIALIZATION_CONSTRAINT_GRAVITY_CONSTRAINT_S11N_HPP
