#ifndef EDYN_SERIALIZATION_COMP_CONTACT_POINT_S11N_HPP
#define EDYN_SERIALIZATION_COMP_CONTACT_POINT_S11N_HPP

#include "edyn/collision/contact_point.hpp"

namespace edyn {

template<typename Archive>
void serialize(Archive &archive, contact_point &cp) {
    archive(cp.body);
    archive(cp.pivotA);
    archive(cp.pivotB);
    archive(cp.normal);
    archive(cp.friction);
    archive(cp.restitution);
    archive(cp.lifetime);
    archive(cp.distance);
}

}

#endif // EDYN_SERIALIZATION_COMP_CONTACT_POINT_S11N_HPP
