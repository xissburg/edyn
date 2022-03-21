#ifndef EDYN_NETWORKING_PACKET_CREATE_ENTITY_HPP
#define EDYN_NETWORKING_PACKET_CREATE_ENTITY_HPP

#include <vector>
#include "edyn/networking/util/pool_snapshot.hpp"

namespace edyn::packet {

/**
 * @brief A request to replicate entities and components.
 */
struct create_entity {
    double timestamp;
    std::vector<entt::entity> entities;
    std::vector<pool_snapshot> pools;
};

template<typename Archive>
void serialize(Archive &archive, create_entity &packet) {
    archive(packet.timestamp);
    archive(packet.entities);
    archive(packet.pools);
}

}

#endif // EDYN_NETWORKING_PACKET_CREATE_ENTITY_HPP
