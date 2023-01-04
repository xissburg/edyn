#ifndef EDYN_NETWORKING_PACKET_ASSET_SYNC_HPP
#define EDYN_NETWORKING_PACKET_ASSET_SYNC_HPP

#include "edyn/networking/packet/registry_snapshot.hpp"
#include <entt/entity/fwd.hpp>
#include <cstdint>

namespace edyn::packet {

struct asset_sync {
    uint32_t id;
    entt::entity entity;
};

template<typename Archive>
void serialize(Archive &archive, asset_sync &packet) {
    archive(packet.id);
    archive(packet.entity);
}

struct asset_sync_response : public registry_snapshot {
    uint32_t id;
    entt::entity entity;
};

template<typename Archive>
void serialize(Archive &archive, asset_sync_response &packet) {
    archive(packet.id);
    archive(packet.entity);
    //archive(packet.timestamp); // Unnecessary in this context.
    archive(packet.entities);
    archive(packet.pools);
}

}

#endif // EDYN_NETWORKING_PACKET_ASSET_SYNC_HPP
