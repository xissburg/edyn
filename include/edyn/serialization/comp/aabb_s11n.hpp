#ifndef EDYN_SERIALIZATION_COMP_AABB_S11N_HPP
#define EDYN_SERIALIZATION_COMP_AABB_S11N_HPP

#include "edyn/comp/aabb.hpp"

namespace edyn {

template<typename Archive>
void serialize(Archive &archive, AABB &aabb) {
    archive(aabb.min);
    archive(aabb.max);
}

}

#endif // EDYN_SERIALIZATION_COMP_AABB_S11N_HPP