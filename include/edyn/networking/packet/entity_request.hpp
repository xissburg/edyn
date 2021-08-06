#ifndef EDYN_NETWORKING_PACKET_ENTITY_REQUEST_HPP
#define EDYN_NETWORKING_PACKET_ENTITY_REQUEST_HPP

#include <vector>
#include <entt/entity/fwd.hpp>

namespace edyn {

struct entity_request {
    std::vector<entt::entity> entities;
};

template<typename Archive>
void serialize(Archive &archive, entity_request &req) {
    archive(req.entities);
}

}

#endif // EDYN_NETWORKING_PACKET_ENTITY_REQUEST_HPP
