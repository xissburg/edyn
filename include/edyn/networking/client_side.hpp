#ifndef EDYN_NETWORKING_CLIENT_SIDE_HPP
#define EDYN_NETWORKING_CLIENT_SIDE_HPP

#include <entt/entity/fwd.hpp>
#include "edyn/networking/packet/edyn_packet.hpp"

namespace edyn {

void init_networking_client(entt::registry &);
void update_networking_client(entt::registry &);
void client_process_packet(entt::registry &, const packet::edyn_packet &);

}

#endif // EDYN_NETWORKING_CLIENT_SIDE_HPP
