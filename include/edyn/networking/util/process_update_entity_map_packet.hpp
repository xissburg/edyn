#ifndef EDYN_NETWORKING_UTIL_PROCESS_UPDATE_ENTITY_MAP_PACKET_HPP
#define EDYN_NETWORKING_UTIL_PROCESS_UPDATE_ENTITY_MAP_PACKET_HPP

#include "edyn/networking/packet/update_entity_map.hpp"
#include "edyn/replication/entity_map.hpp"
#include <entt/entity/fwd.hpp>

namespace edyn {

void process_update_entity_map_packet(const entt::registry &, const packet::update_entity_map &, entity_map &);

}

#endif // EDYN_NETWORKING_UTIL_PROCESS_UPDATE_ENTITY_MAP_PACKET_HPP
