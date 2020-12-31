#ifndef EDYN_SERIALIZATION_COMP_INERTIA_S11N_HPP
#define EDYN_SERIALIZATION_COMP_INERTIA_S11N_HPP

#include "edyn/comp/inertia.hpp"

namespace edyn {

template<typename Archive>
void serialize(Archive &archive, inertia &i) {
    archive(i.x, i.y, i.z);
}

template<typename Archive>
void serialize(Archive &archive, inertia_inv &i) {
    archive(i.x, i.y, i.z);
}

template<typename Archive>
void serialize(Archive &archive, inertia_world_inv &i) {
    archive(i.row);
}

}

#endif // EDYN_SERIALIZATION_COMP_INERTIA_S11N_HPP