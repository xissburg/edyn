#ifndef EDYN_NETWORKING_PACKET_CREATE_ENTITY_HPP
#define EDYN_NETWORKING_PACKET_CREATE_ENTITY_HPP

#include <vector>
#include "edyn/networking/packet/util/entity_components_pair.hpp"

namespace edyn::packet {

struct create_entity {
    std::vector<entity_components_pair> pairs;
};

template<typename Archive>
void serialize(Archive &archive, create_entity &packet) {
    archive(packet.pairs);
}

}

#endif // EDYN_NETWORKING_PACKET_CREATE_ENTITY_HPP
