#ifndef EDYN_NETWORKING_NETWORKING_HPP
#define EDYN_NETWORKING_NETWORKING_HPP

#include "edyn/networking/packet/packet.hpp"
#include <entt/entity/fwd.hpp>

namespace edyn {

void init_networking_server(entt::registry &);

void init_networking_client(entt::registry &);

}

#endif // EDYN_NETWORKING_NETWORKING_HPP
