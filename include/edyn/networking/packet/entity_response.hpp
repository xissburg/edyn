#ifndef EDYN_NETWORKING_PACKET_ENTITY_RESPONSE_HPP
#define EDYN_NETWORKING_PACKET_ENTITY_RESPONSE_HPP

#include <vector>
#include "edyn/networking/packet/util/pool_snapshot.hpp"

namespace edyn::packet {

struct entity_response {
    std::vector<entt::entity> entities;
    std::vector<pool_snapshot> pools;
};

template<typename Archive>
void serialize(Archive &archive, entity_response &res) {
    archive(res.entities);
    archive(res.pools);
}

}

#endif // EDYN_NETWORKING_PACKET_ENTITY_RESPONSE_HPP