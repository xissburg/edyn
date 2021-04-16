#ifndef EDYN_SERIALIZATION_CONSTRAINT_DISTANCE_CONSTRAINT_S11N_HPP
#define EDYN_SERIALIZATION_CONSTRAINT_DISTANCE_CONSTRAINT_S11N_HPP

#include "edyn/constraints/distance_constraint.hpp"

namespace edyn {

template<typename Archive>
void serialize(Archive &archive, distance_constraint &c) {
    archive(c.body);
    archive(c.impulse);
    archive(c.pivot);
    archive(c.distance);
}

}

#endif // EDYN_SERIALIZATION_CONSTRAINT_DISTANCE_CONSTRAINT_S11N_HPP