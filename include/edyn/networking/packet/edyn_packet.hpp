#ifndef EDYN_NETWORKING_PACKET_EDYN_PACKET_HPP
#define EDYN_NETWORKING_PACKET_EDYN_PACKET_HPP

#include "edyn/networking/packet/entity_request.hpp"
#include "edyn/networking/packet/entity_response.hpp"
#include "edyn/networking/packet/transient_snapshot.hpp"
#include "edyn/networking/packet/create_entity.hpp"
#include "edyn/networking/packet/destroy_entity.hpp"
#include "edyn/networking/packet/update_entity_map.hpp"
#include "edyn/networking/packet/client_created.hpp"
#include "edyn/networking/packet/general_snapshot.hpp"
#include "edyn/networking/packet/set_playout_delay.hpp"
#include "edyn/networking/packet/time_request.hpp"
#include "edyn/networking/packet/time_response.hpp"
#include <variant>

namespace edyn::packet {

struct edyn_packet {
    std::variant<
        entity_request,
        entity_response,
        transient_snapshot,
        create_entity,
        destroy_entity,
        update_entity_map,
        client_created,
        general_snapshot,
        set_playout_delay,
        time_request,
        time_response
    > var;
};

using timed_packets_tuple_t = std::tuple<
    packet::transient_snapshot,
    packet::general_snapshot,
    packet::create_entity,
    packet::destroy_entity,
    packet::update_entity_map
>;

template<typename Archive>
void serialize(Archive &archive, edyn_packet &packet) {
    archive(packet.var);
}

}

#endif // EDYN_NETWORKING_PACKET_EDYN_PACKET_HPP
