#ifndef EDYN_NETWORKING_PACKET_CLIENT_CREATED_HPP
#define EDYN_NETWORKING_PACKET_CLIENT_CREATED_HPP

#include <entt/entity/fwd.hpp>

namespace edyn::packet {

/**
 * @brief Sent from server to client once the client entity is created after
 * connection is established.
 */
struct client_created {
    entt::entity client_entity;
};

template<typename Archive>
void serialize(Archive &archive, client_created &packet) {
    archive(packet.client_entity);
}

}

#endif // EDYN_NETWORKING_PACKET_CLIENT_CREATED_HPP
