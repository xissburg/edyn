#ifndef EDYN_NETWORKING_REMOTE_CLIENT_HPP
#define EDYN_NETWORKING_REMOTE_CLIENT_HPP

#include <entt/signal/sigh.hpp>
#include "edyn/networking/packet/packet.hpp"

namespace edyn {

struct remote_client {
    using packet_observer_func_t = void(const edyn_packet &);
    entt::sigh<packet_observer_func_t> packet_signal;

    auto packet_sink() {
        return entt::sink{packet_signal};
    }
};

}

#endif // EDYN_NETWORKING_REMOTE_CLIENT_HPP
