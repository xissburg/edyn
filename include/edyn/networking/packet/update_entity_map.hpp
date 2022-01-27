#ifndef EDYN_NETWORKING_PACKET_UPDATE_ENTITY_MAP_HPP
#define EDYN_NETWORKING_PACKET_UPDATE_ENTITY_MAP_HPP

#include "edyn/util/entity_pair.hpp"

namespace edyn::packet {

struct update_entity_map {
    entity_pair_vector pairs; // [local -> remote]
};

template<typename Archive>
void serialize(Archive &archive, update_entity_map &map) {
    archive(map.pairs);
}

}

#endif // EDYN_NETWORKING_PACKET_UPDATE_ENTITY_MAP_HPP