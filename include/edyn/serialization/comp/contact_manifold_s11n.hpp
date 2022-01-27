#ifndef EDYN_SERIALIZATION_COMP_CONTACT_MANIFOLD_S11N_HPP
#define EDYN_SERIALIZATION_COMP_CONTACT_MANIFOLD_S11N_HPP

#include "edyn/collision/contact_manifold.hpp"

namespace edyn {

template<typename Archive>
void serialize(Archive &archive, contact_manifold &manifold) {
    archive(manifold.body);
    archive(manifold.separation_threshold);
    archive(manifold.num_points);

    for (unsigned i = 0; i < manifold.num_points; ++i) {
        archive(manifold.ids[i]);
    }

    manifold.each_point([&] (contact_point &cp) {
        archive(cp);
    });
}

template<typename Archive>
void serialize(Archive &, contact_manifold_with_restitution &) {}

}

#endif // EDYN_SERIALIZATION_COMP_CONTACT_MANIFOLD_S11N_HPP
