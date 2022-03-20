#ifndef EDYN_NETWORKING_SYS_SERVER_VALIDATE_PACKET_HPP
#define EDYN_NETWORKING_SYS_SERVER_VALIDATE_PACKET_HPP

#include <entt/entity/fwd.hpp>
#include "edyn/networking/packet/edyn_packet.hpp"

namespace edyn {

bool server_validate_packet(const entt::registry &registry, entt::entity client_entity, packet::entity_response &packet);
bool server_validate_packet(const entt::registry &registry, entt::entity client_entity, packet::create_entity &packet);
bool server_validate_packet(const entt::registry &registry, entt::entity client_entity, packet::destroy_entity &packet);
bool server_validate_packet(const entt::registry &registry, entt::entity client_entity, packet::update_entity_map &packet);
bool server_validate_packet(const entt::registry &registry, entt::entity client_entity, packet::client_created &packet);
bool server_validate_packet(const entt::registry &registry, entt::entity client_entity, packet::set_playout_delay &packet);
bool server_validate_packet(const entt::registry &registry, entt::entity client_entity, packet::time_request &packet);
bool server_validate_packet(const entt::registry &registry, entt::entity client_entity, packet::time_response &packet);
bool server_validate_packet(const entt::registry &registry, entt::entity client_entity, packet::entity_request &packet);
bool server_validate_packet(const entt::registry &registry, entt::entity client_entity, packet::transient_snapshot &packet);
bool server_validate_packet(const entt::registry &registry, entt::entity client_entity, packet::general_snapshot &packet);

}

#endif // EDYN_NETWORKING_SYS_SERVER_VALIDATE_PACKET_HPP
