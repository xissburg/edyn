#include "edyn/networking/util/process_update_entity_map_packet.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

void process_update_entity_map_packet(const entt::registry &registry, const packet::update_entity_map &packet, entity_map &emap) {
    for (auto &pair : packet.pairs) {
        auto local_entity = pair.first;
        auto remote_entity = pair.second;

        if (!registry.valid(local_entity)) {
            continue;
        }

        if (emap.has_loc(local_entity)) {
            // If there is a mapping for the local entity but the received remote
            // entity is different, replace it.
            if (emap.locrem(local_entity) != remote_entity) {
                emap.erase_loc(local_entity);
                emap.insert(remote_entity, local_entity);
            }
        } else if (emap.has_rem(remote_entity)) {
            // If there is a mapping for the remote entity, replace it with the
            // received local entity.
            emap.erase_rem(remote_entity);
            emap.insert(remote_entity, local_entity);
        } else {
            emap.insert(remote_entity, local_entity);
        }
    }
}

}
