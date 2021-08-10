#ifndef EDYN_NETWORKING_REMOTE_CLIENT_HPP
#define EDYN_NETWORKING_REMOTE_CLIENT_HPP

#include <vector>
#include <entt/entity/fwd.hpp>
#include <entt/signal/sigh.hpp>
#include "edyn/util/entity_map.hpp"

namespace edyn {

struct edyn_packet;

struct remote_client {
    using packet_observer_func_t = void(const edyn_packet &);
    entt::sigh<packet_observer_func_t> packet_signal;

    auto packet_sink() {
        return entt::sink{packet_signal};
    }

    std::vector<entt::entity> owned_entities;
    edyn::entity_map entity_map;
};

}

#endif // EDYN_NETWORKING_REMOTE_CLIENT_HPP
