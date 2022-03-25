#ifndef EDYN_NETWORKING_PACKET_UPDATE_ENTITY_MAP_HPP
#define EDYN_NETWORKING_PACKET_UPDATE_ENTITY_MAP_HPP

#include "edyn/util/entity_pair.hpp"

namespace edyn::packet {

/**
 * @brief Produced whenever a local entity is created to correspond to a remote
 * entity. It informs the other end into which remote entity their local entity
 * has been mapped.
 */
struct update_entity_map {
    double timestamp;
    entity_pair_vector pairs; // [local -> remote]
};

template<typename Archive>
void serialize(Archive &archive, update_entity_map &map) {
    archive(map.timestamp);
    archive(map.pairs);
}

}

#endif // EDYN_NETWORKING_PACKET_UPDATE_ENTITY_MAP_HPP
