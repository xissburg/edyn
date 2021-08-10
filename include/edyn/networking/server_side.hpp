#ifndef EDYN_NETWORKING_SERVER_SIDE_HPP
#define EDYN_NETWORKING_SERVER_SIDE_HPP

#include <entt/entity/fwd.hpp>
#include "edyn/networking/packet/edyn_packet.hpp"

namespace edyn {

void init_networking_server(entt::registry &);

void server_process_packet(entt::registry &, entt::entity client_entity, const packet::edyn_packet &);

packet::transient_snapshot server_get_transient_snapshot(entt::registry &);

void server_make_client(entt::registry &, entt::entity);
entt::entity server_make_client(entt::registry &);

}

#endif // EDYN_NETWORKING_SERVER_SIDE_HPP
