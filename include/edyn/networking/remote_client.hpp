#ifndef EDYN_NETWORKING_REMOTE_CLIENT_HPP
#define EDYN_NETWORKING_REMOTE_CLIENT_HPP

#include <vector>
#include <entt/entity/fwd.hpp>
#include <entt/signal/sigh.hpp>
#include "edyn/util/entity_map.hpp"
#include "edyn/networking/packet/edyn_packet.hpp"

namespace edyn {

struct remote_client {
    using packet_observer_func_t = void(entt::entity, const packet::edyn_packet &);
    entt::sigh<packet_observer_func_t> packet_signal;

    auto packet_sink() {
        return entt::sink{packet_signal};
    }

    std::vector<entt::entity> owned_entities;
    edyn::entity_map entity_map;
    double latency {0};
    double playout_delay {0};
    std::vector<edyn::packet::edyn_packet> packet_queue;

    double last_snapshot_time {0};
    double snapshot_rate {1};
};

}

#endif // EDYN_NETWORKING_REMOTE_CLIENT_HPP
