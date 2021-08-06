#ifndef EDYN_SERIALIZATION_NULL_CONSTRAINT_S11N_HPP
#define EDYN_SERIALIZATION_NULL_CONSTRAINT_S11N_HPP

#include "edyn/constraints/null_constraint.hpp"

namespace edyn {

template<typename Archive>
void serialize(Archive &archive, null_constraint &con) {
    archive(con.body);
}

}

#endif // EDYN_SERIALIZATION_NULL_CONSTRAINT_S11N_HPP
