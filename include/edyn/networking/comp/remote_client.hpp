#ifndef EDYN_NETWORKING_REMOTE_CLIENT_HPP
#define EDYN_NETWORKING_REMOTE_CLIENT_HPP

#include <vector>
#include <entt/entity/fwd.hpp>
#include <entt/entity/sparse_set.hpp>
#include "edyn/replication/entity_map.hpp"
#include "edyn/networking/packet/edyn_packet.hpp"
#include "edyn/networking/util/clock_sync.hpp"

namespace edyn {

struct timed_packet {
    double timestamp;
    packet::edyn_packet packet;
};

/**
 * @brief Stores data pertaining to a remote client in the server side.
 */
struct remote_client {
    // List of entities owned by this client.
    entt::sparse_set owned_entities;

    // Maps entities between the client registry and the server registry.
    edyn::entity_map entity_map;

    // Client round-trip time in seconds.
    double round_trip_time {};

    // The delay in seconds applied to packet processing.
    double playout_delay {};

    // List of delayed packets pending processing.
    std::vector<timed_packet> packet_queue;

    // Timestamp of the last registry snapshot that was sent.
    double last_snapshot_time {0};

    // Rate of registry snapshots, i.e. registry snapshots sent per second.
    double snapshot_rate {10};

    // Whether this client will be given temporary ownership of all entities in
    // the island where entities owned by it reside, thus allowing the state of
    // those entities to be set by the client.
    bool allow_full_ownership {true};

    clock_sync_data clock_sync;

    double last_executed_history_entry_timestamp {0};
};

}

#endif // EDYN_NETWORKING_REMOTE_CLIENT_HPP
