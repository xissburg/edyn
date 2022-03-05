#ifndef EDYN_NETWORKING_REMOTE_CLIENT_HPP
#define EDYN_NETWORKING_REMOTE_CLIENT_HPP

#include <vector>
#include <entt/entity/fwd.hpp>
#include <entt/signal/sigh.hpp>
#include "edyn/util/entity_map.hpp"
#include "edyn/networking/packet/edyn_packet.hpp"

namespace edyn {

/**
 * @brief Stores data pertaining to a remote client in the server side.
 */
struct remote_client {
    using packet_observer_func_t = void(entt::entity, const packet::edyn_packet &);

    // Triggered every time the server needs to send a packet to a client.
    // Users of Edyn must observe this signal and send the packet data over
    // the network using their mechanism of choice.
    entt::sigh<packet_observer_func_t> packet_signal;

    auto packet_sink() {
        return entt::sink{packet_signal};
    }

    // List of entities owned by this client.
    std::vector<entt::entity> owned_entities;

    // Maps entities between the client registry and the server registry.
    edyn::entity_map entity_map;

    // Client latency in seconds.
    double latency {};

    // The delay in seconds applied to packet processing.
    double playout_delay {};

    // Difference between local server time and remote client time.
    double time_delta {};

    bool is_calculating_time_delta {false};

    // List of delayed packets pending processing.
    std::vector<packet::edyn_packet> packet_queue;

    // Timestamp of the last transient snapshot that was sent.
    double last_snapshot_time {0};

    // Rate of transient snapshots, i.e. transient snapshots sent per second.
    double snapshot_rate {1};

    // A snapshot where changes to be reported to a client can be accumulated
    // and then consumed at the end of an update.
    packet::general_snapshot current_snapshot;
};

}

#endif // EDYN_NETWORKING_REMOTE_CLIENT_HPP
