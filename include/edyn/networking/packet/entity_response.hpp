#ifndef EDYN_NETWORKING_PACKET_ENTITY_RESPONSE_HPP
#define EDYN_NETWORKING_PACKET_ENTITY_RESPONSE_HPP

#include "edyn/networking/packet/registry_snapshot.hpp"
#include <cstdint>

namespace edyn::packet {

/**
 * @brief A response to an entity query.
 */
struct entity_response : public registry_snapshot {
    uint32_t id;
};

template<typename Archive>
void serialize(Archive &archive, entity_response &packet) {
    archive(packet.id);
    //archive(packet.timestamp); // Unnecessary in this context.
    archive(packet.entities);
    archive(packet.pools);
}

}

#endif // EDYN_NETWORKING_PACKET_ENTITY_RESPONSE_HPP
