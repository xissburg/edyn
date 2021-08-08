#ifndef EDYN_NETWORKING_SERVER_SIDE_HPP
#define EDYN_NETWORKING_SERVER_SIDE_HPP

#include <entt/entity/fwd.hpp>
#include "edyn/networking/packet/packet.hpp"

namespace edyn {

void server_process_packet(entt::registry &, entt::entity client_entity, const edyn_packet &);

transient_snapshot server_get_transient_snapshot(entt::registry &);

}

#endif // EDYN_NETWORKING_SERVER_SIDE_HPP
