#ifndef EDYN_NETWORKING_PACKET_ENTITY_EXITED_HPP
#define EDYN_NETWORKING_PACKET_ENTITY_EXITED_HPP

#include <entt/entity/fwd.hpp>
#include <vector>

namespace edyn::packet {

struct entity_exited {
    std::vector<entt::entity> entities;
};

template<typename Archive>
void serialize(Archive &archive, entity_exited &packet) {
    archive(packet.entities);
}

}

#endif // EDYN_NETWORKING_PACKET_ENTITY_EXITED_HPP
