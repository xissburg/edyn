#ifndef EDYN_NETWORKING_PACKET_QUERY_ENTITY_HPP
#define EDYN_NETWORKING_PACKET_QUERY_ENTITY_HPP

#include "edyn/networking/util/component_index_type.hpp"
#include <entt/entity/fwd.hpp>
#include <utility>
#include <vector>

namespace edyn::packet {

struct query_entity {
    uint32_t id;
    // Vector of pairs of entity and desired components to query.
    std::vector<std::pair<entt::entity, std::vector<component_index_type>>> entities;
};

template<typename Archive>
void serialize(Archive &archive, query_entity &packet) {
    archive(packet.entities);
}

}

#endif // EDYN_NETWORKING_PACKET_QUERY_ENTITY_HPP
