#ifndef EDYN_NETWORKING_PACKET_ENTITY_REQUEST_HPP
#define EDYN_NETWORKING_PACKET_ENTITY_REQUEST_HPP

#include <vector>
#include <entt/entity/fwd.hpp>

namespace edyn::packet {

/**
 * @brief Request information of entities that are not yet known.
 * An `entity_response` is expected later, containing the components of the
 * requested entities.
 */
struct entity_request {
    std::vector<entt::entity> entities;
};

template<typename Archive>
void serialize(Archive &archive, entity_request &req) {
    archive(req.entities);
}

}

#endif // EDYN_NETWORKING_PACKET_ENTITY_REQUEST_HPP
