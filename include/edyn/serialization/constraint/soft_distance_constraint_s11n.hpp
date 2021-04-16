#ifndef EDYN_SERIALIZATION_CONSTRAINT_SOFT_DISTANCE_CONSTRAINT_S11N_HPP
#define EDYN_SERIALIZATION_CONSTRAINT_SOFT_DISTANCE_CONSTRAINT_S11N_HPP

#include "edyn/constraints/soft_distance_constraint.hpp"

namespace edyn {

template<typename Archive>
void serialize(Archive &archive, soft_distance_constraint &c) {
    archive(c.body);
    archive(c.pivot);
    archive(c.distance);
    archive(c.stiffness);
    archive(c.damping);
    archive(c.m_relspd);
}

}


#endif // EDYN_SERIALIZATION_CONSTRAINT_SOFT_DISTANCE_CONSTRAINT_S11N_HPP