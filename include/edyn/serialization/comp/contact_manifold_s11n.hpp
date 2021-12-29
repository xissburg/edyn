#ifndef EDYN_SERIALIZATION_COMP_CONTACT_MANIFOLD_S11N_HPP
#define EDYN_SERIALIZATION_COMP_CONTACT_MANIFOLD_S11N_HPP

#include "edyn/collision/contact_manifold.hpp"

namespace edyn {

template<typename Archive>
void serialize(Archive &archive, contact_manifold &manifold) {
    archive(manifold.body);
    archive(manifold.separation_threshold);
    archive(manifold.point);
}

template<typename Archive>
void serialize(Archive &, contact_manifold_with_restitution &) {}

}

#endif // EDYN_SERIALIZATION_COMP_CONTACT_MANIFOLD_S11N_HPP
