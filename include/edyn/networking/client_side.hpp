#ifndef EDYN_NETWORKING_CLIENT_SIDE_HPP
#define EDYN_NETWORKING_CLIENT_SIDE_HPP

#include <entt/entity/fwd.hpp>
#include "edyn/networking/packet/packet.hpp"

namespace edyn {

void client_process_packet(entt::registry &, const edyn_packet &);

}

#endif // EDYN_NETWORKING_CLIENT_SIDE_HPP
