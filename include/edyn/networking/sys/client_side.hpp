#ifndef EDYN_NETWORKING_CLIENT_SIDE_HPP
#define EDYN_NETWORKING_CLIENT_SIDE_HPP

#include <entt/entity/fwd.hpp>
#include "edyn/networking/packet/edyn_packet.hpp"

namespace edyn {

void init_networking_client(entt::registry &);
void deinit_networking_client(entt::registry &);
void update_networking_client(entt::registry &);
void client_handle_packet(entt::registry &, const packet::edyn_packet &);

entt::sink<void()> on_client_entity_assigned(entt::registry &);
bool client_owns_entity(entt::registry &, entt::entity);

}

#endif // EDYN_NETWORKING_CLIENT_SIDE_HPP
