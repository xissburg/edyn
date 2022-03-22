#ifndef EDYN_NETWORKING_PACKET_ENTITY_RESPONSE_HPP
#define EDYN_NETWORKING_PACKET_ENTITY_RESPONSE_HPP

#include <vector>
#include "edyn/networking/util/registry_snapshot.hpp"

namespace edyn::packet {

/**
 * @brief A response to an entity request.
 */
struct entity_response : public registry_snapshot {};

template<typename Archive>
void serialize(Archive &archive, entity_response &res) {
    archive(res.entities);
    archive(res.pools);
}

}

#endif // EDYN_NETWORKING_PACKET_ENTITY_RESPONSE_HPP
