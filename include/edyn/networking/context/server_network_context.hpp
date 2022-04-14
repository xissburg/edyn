#ifndef EDYN_NETWORKING_SERVER_NETWORK_CONTEXT_HPP
#define EDYN_NETWORKING_SERVER_NETWORK_CONTEXT_HPP

#include <cstdint>
#include <vector>
#include <entt/entity/fwd.hpp>
#include <entt/signal/sigh.hpp>
#include "edyn/networking/util/server_snapshot_importer.hpp"
#include "edyn/networking/util/server_snapshot_exporter.hpp"

namespace edyn {

struct server_network_context {
    server_network_context();

    std::vector<entt::entity> pending_created_clients;

    std::shared_ptr<server_snapshot_importer> snapshot_importer;
    std::shared_ptr<server_snapshot_exporter> snapshot_exporter;

    // Packet signals contain the client entity and the packet.
    using packet_observer_func_t = void(entt::entity, const packet::edyn_packet &);
    entt::sigh<packet_observer_func_t> packet_signal;

    auto packet_sink() {
        return entt::sink{packet_signal};
    }

    using action_observer_func_t = void(entt::entity, const std::vector<uint8_t> &);
    entt::sigh<action_observer_func_t> action_signal;

    auto action_sink() {
        return entt::sink{action_signal};
    }
};

}

#endif // EDYN_NETWORKING_SERVER_NETWORK_CONTEXT_HPP
