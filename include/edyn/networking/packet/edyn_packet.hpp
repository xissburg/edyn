#ifndef EDYN_NETWORKING_PACKET_EDYN_PACKET_HPP
#define EDYN_NETWORKING_PACKET_EDYN_PACKET_HPP

#include "edyn/networking/packet/entity_request.hpp"
#include "edyn/networking/packet/entity_response.hpp"
#include "edyn/networking/packet/transient_snapshot.hpp"
#include "edyn/networking/packet/create_entity.hpp"
#include "edyn/networking/packet/update_entity_map.hpp"
#include <variant>

namespace edyn::packet {

struct edyn_packet {
    std::variant<
        entity_request,
        entity_response,
        transient_snapshot,
        create_entity,
        update_entity_map
    > var;
};

template<typename Archive>
void serialize(Archive &archive, edyn_packet &packet) {
    archive(packet.var);
}

}

#endif // EDYN_NETWORKING_PACKET_EDYN_PACKET_HPP
