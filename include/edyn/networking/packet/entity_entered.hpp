#ifndef EDYN_NETWORKING_PACKET_ENTITY_ENTERED_HPP
#define EDYN_NETWORKING_PACKET_ENTITY_ENTERED_HPP

#include "edyn/networking/comp/asset_ref.hpp"
#include "edyn/networking/packet/registry_snapshot.hpp"
#include "edyn/networking/util/pool_snapshot.hpp"
#include <entt/entity/fwd.hpp>
#include <vector>

namespace edyn::packet {

struct entity_entered {
    struct asset_info : public registry_snapshot {
        entt::entity entity;
        asset_ref asset;
        entt::entity owner;
    };
    std::vector<asset_info> entry;
};

template<typename Archive>
void serialize(Archive &archive, entity_entered::asset_info &info) {
    archive(info.entity);
    archive(info.asset);
    archive(info.owner);
    archive(info.entities);
    archive(info.pools);
}

template<typename Archive>
void serialize(Archive &archive, entity_entered &packet) {
    archive(packet.entry);
}

}

#endif // EDYN_NETWORKING_PACKET_ENTITY_ENTERED_HPP
