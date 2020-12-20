#ifndef EDNY_SERIALIZATION_COMP_COLLISION_FILTER_S11N_HPP
#define EDNY_SERIALIZATION_COMP_COLLISION_FILTER_S11N_HPP

#include "edyn/comp/collision_filter.hpp"

namespace edyn {

template<typename Archive>
void serialize(Archive &archive, collision_filter &c) {
    archive(c.group);
    archive(c.mask);
}

}

#endif // EDNY_SERIALIZATION_COMP_COLLISION_FILTER_S11N_HPP