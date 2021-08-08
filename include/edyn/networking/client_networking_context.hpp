#ifndef EDYN_NETWORKING_CLIENT_NETWORKING_CONTEXT_HPP
#define EDYN_NETWORKING_CLIENT_NETWORKING_CONTEXT_HPP

#include "edyn/util/entity_map.hpp"
#include <entt/signal/sigh.hpp>

namespace edyn {

struct client_networking_context {
    edyn::entity_map entity_map;

    using request_entity_func_t = void(entt::entity);
    entt::sigh<request_entity_func_t> request_entity_signal;

    auto request_entity_sink() {
        return entt::sink{request_entity_signal};
    }
};

}

#endif // EDYN_NETWORKING_CLIENT_NETWORKING_CONTEXT_HPP
