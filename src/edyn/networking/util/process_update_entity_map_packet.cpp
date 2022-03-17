#include "edyn/networking/util/process_update_entity_map_packet.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

void process_update_entity_map_packet(const entt::registry &registry, const packet::update_entity_map &packet, entity_map &emap) {
    for (auto &pair : packet.pairs) {
        auto local_entity = pair.first;
        auto remote_entity = pair.second;

        if (registry.valid(local_entity)) {
            emap.insert(remote_entity, local_entity);
        }
    }
}

}
