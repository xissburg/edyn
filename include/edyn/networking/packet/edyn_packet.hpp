#ifndef EDYN_NETWORKING_PACKET_EDYN_PACKET_HPP
#define EDYN_NETWORKING_PACKET_EDYN_PACKET_HPP

#include "edyn/networking/packet/asset_sync.hpp"
#include "edyn/networking/packet/entity_entered.hpp"
#include "edyn/networking/packet/entity_exited.hpp"
#include "edyn/networking/packet/entity_response.hpp"
#include "edyn/networking/packet/query_entity.hpp"
#include "edyn/networking/packet/registry_snapshot.hpp"
#include "edyn/networking/packet/create_entity.hpp"
#include "edyn/networking/packet/destroy_entity.hpp"
#include "edyn/networking/packet/update_entity_map.hpp"
#include "edyn/networking/packet/client_created.hpp"
#include "edyn/networking/packet/set_playout_delay.hpp"
#include "edyn/networking/packet/time_request.hpp"
#include "edyn/networking/packet/time_response.hpp"
#include "edyn/networking/packet/server_settings.hpp"
#include "edyn/networking/packet/set_aabb_of_interest.hpp"
#include <variant>

namespace edyn::packet {

/**
 * @brief The Edyn packet which holds a payload of many internal packet types.
 * Applications must add this packet as one of their own packet types and when
 * a packet of this type is received, it must be fed into the Edyn engine.
 */
struct edyn_packet {
    std::variant<
        registry_snapshot,
        create_entity,
        destroy_entity,
        update_entity_map,
        client_created,
        set_playout_delay,
        time_request,
        time_response,
        server_settings,
        set_aabb_of_interest,
        query_entity,
        entity_response,
        entity_entered,
        entity_exited,
        asset_sync,
        asset_sync_response
    > var;
};

using timed_packets_tuple_t = std::tuple<
    packet::registry_snapshot,
    packet::create_entity,
    packet::destroy_entity,
    packet::update_entity_map
>;

using unreliable_packets_tuple_t = std::tuple<
    packet::registry_snapshot,
    packet::time_request,
    packet::time_response
>;

template<typename Archive>
void serialize(Archive &archive, edyn_packet &packet) {
    archive(packet.var);
}

}

namespace edyn {

/**
 * @brief Determine whether a packet should be sent reliably, thus having its
 * delivery guaranteed. Otherwise, packet loss is acceptable and reliability is
 * not necessary.
 * @param packet The Edyn packet.
 * @return True if the packet delivery must be guaranteed.
 */
inline bool should_send_reliably(const packet::edyn_packet &packet) {
    bool result;
    std::visit([&result](auto &&inner_packet) {
        using PacketType = std::decay_t<decltype(inner_packet)>;
        result = !tuple_has_type<PacketType, packet::unreliable_packets_tuple_t>::value;
    }, packet.var);
    return result;
}

}

#endif // EDYN_NETWORKING_PACKET_EDYN_PACKET_HPP
