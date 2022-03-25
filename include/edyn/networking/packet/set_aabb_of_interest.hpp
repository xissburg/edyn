#ifndef EDYN_NETWORKING_PACKET_SET_AABB_OF_INTEREST_HPP
#define EDYN_NETWORKING_PACKET_SET_AABB_OF_INTEREST_HPP

#include "edyn/math/vector3.hpp"

namespace edyn::packet {

struct set_aabb_of_interest {
    vector3 min, max;
};

template<typename Archive>
void serialize(Archive &archive, set_aabb_of_interest &aabb) {
    archive(aabb.min, aabb.max);
}

}

#endif // EDYN_NETWORKING_PACKET_SET_AABB_OF_INTEREST_HPP
