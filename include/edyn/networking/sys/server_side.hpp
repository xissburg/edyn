#ifndef EDYN_NETWORKING_SERVER_SIDE_HPP
#define EDYN_NETWORKING_SERVER_SIDE_HPP

#include <entt/entity/fwd.hpp>
#include "edyn/networking/packet/edyn_packet.hpp"

namespace edyn {

void init_network_server(entt::registry &);
void deinit_network_server(entt::registry &);
void server_process_packets(entt::registry &);
void update_network_server(entt::registry &);

void server_handle_packet(entt::registry &, entt::entity client_entity, const packet::edyn_packet &);

void server_make_client(entt::registry &, entt::entity);
entt::entity server_make_client(entt::registry &);

void server_set_client_latency(entt::registry &, entt::entity client_entity, double latency);

void server_assign_ownership_to_client(entt::registry &registry, entt::entity client_entity, std::vector<entt::entity> &entities);

}

#endif // EDYN_NETWORKING_SERVER_SIDE_HPP
