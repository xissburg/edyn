#ifndef EDYN_NETWORKING_PACKET_ENTITY_RESPONSE_HPP
#define EDYN_NETWORKING_PACKET_ENTITY_RESPONSE_HPP

#include <vector>
#include "edyn/networking/packet/util/entity_components_pair.hpp"

namespace edyn::packet {

struct entity_response {
    std::vector<entity_components_pair> pairs;
};

template<typename Archive>
void serialize(Archive &archive, entity_response &res) {
    archive(res.pairs);
}

}

#endif // EDYN_NETWORKING_PACKET_ENTITY_RESPONSE_HPP
