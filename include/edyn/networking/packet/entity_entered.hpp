#ifndef EDYN_NETWORKING_PACKET_ENTITY_ENTERED_HPP
#define EDYN_NETWORKING_PACKET_ENTITY_ENTERED_HPP

#include "edyn/networking/comp/asset_ref.hpp"
#include <entt/entity/fwd.hpp>
#include <vector>

namespace edyn::packet {

struct entity_entered {
    std::vector<entt::entity> entities;
    std::vector<asset_ref> assets;
};

template<typename Archive>
void serialize(Archive &archive, entity_entered &packet) {
    archive(packet.entities);
    archive(packet.assets);
}

}

#endif // EDYN_NETWORKING_PACKET_ENTITY_ENTERED_HPP
