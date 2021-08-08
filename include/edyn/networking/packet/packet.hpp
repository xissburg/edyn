#ifndef EDYN_NETWORKING_PACKET_PACKET_HPP
#define EDYN_NETWORKING_PACKET_PACKET_HPP

#include "edyn/networking/packet/entity_request.hpp"
#include "edyn/networking/packet/entity_response.hpp"
#include "edyn/networking/packet/pool_snapshot.hpp"
#include "edyn/networking/packet/transient_snapshot.hpp"
#include <variant>

namespace edyn {

struct edyn_packet {
    std::variant<
        entity_request,
        entity_response,
        transient_snapshot
    > var;
};

template<typename Archive>
void serialize(Archive &archive, edyn_packet &packet) {
    archive(packet.var);
}

}

#endif // EDYN_NETWORKING_PACKET_PACKET_HPP
