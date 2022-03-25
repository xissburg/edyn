#ifndef EDYN_NETWORKING_PACKET_DESTROY_ENTITY_HPP
#define EDYN_NETWORKING_PACKET_DESTROY_ENTITY_HPP

#include <vector>
#include <entt/entity/fwd.hpp>

namespace edyn::packet {

/**
 * @brief A request to destroy entities.
 */
struct destroy_entity {
    double timestamp;
    std::vector<entt::entity> entities;
};

template<typename Archive>
void serialize(Archive &archive, destroy_entity &packet) {
    archive(packet.timestamp);
    archive(packet.entities);
}

}

#endif // EDYN_NETWORKING_PACKET_DESTROY_ENTITY_HPP
