#ifndef EDYN_SERIALIZATION_CONSTRAINT_POINT_CONSTRAINT_S11N_HPP
#define EDYN_SERIALIZATION_CONSTRAINT_POINT_CONSTRAINT_S11N_HPP

#include "edyn/constraints/point_constraint.hpp"

namespace edyn {

template<typename Archive>
void serialize(Archive &archive, point_constraint &c) {
    archive(c.pivot);
}

}

#endif // EDYN_SERIALIZATION_CONSTRAINT_POINT_CONSTRAINT_S11N_HPP