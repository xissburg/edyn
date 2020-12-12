#include "edyn/parallel/registry_delta.hpp"
#include <entt/entt.hpp>

namespace edyn {

void registry_delta::import_created_entities(entt::registry &registry, entity_map &map) const {
    for (auto remote_entity : m_created_entities) {
        if (map.has_rem(remote_entity)) continue;
        auto local_entity = registry.create();
        map.insert(remote_entity, local_entity);
    }
}

void registry_delta::import_destroyed_entities(entt::registry &registry, entity_map &map) const {
    for (auto remote_entity : m_destroyed_entities) {
        if (!map.has_rem(remote_entity)) continue;
        auto local_entity = map.remloc(remote_entity);
        map.erase_rem(remote_entity);

        if (registry.valid(local_entity)) {
            registry.destroy(local_entity);
        }
    }
}

void registry_delta::import(entt::registry &registry, entity_map &map) const {
    m_entity_map.each([&registry, &map] (entt::entity remote_entity, entt::entity local_entity) {
        if (!map.has_rem(remote_entity) && registry.valid(local_entity)) {
            map.insert(remote_entity, local_entity);
        }
    });
    
    import_created_entities(registry, map);
    import_destroyed_components(registry, map, all_components{});
    import_destroyed_entities(registry, map);

    import_created_components(registry, map, all_components{});
    import_updated(registry, map, all_components{});
}

void registry_delta_builder::insert_entity_mapping(entt::entity local_entity) {
    // Note that this is being called from the builder and the order is reversed,
    // i.e. (local, remote). When importing, the "correct" order is used, so the
    // first entity which is the remote, refers to the local entity in this registry.
    auto remote_entity = m_entity_map->locrem(local_entity);
    m_delta.m_entity_map.insert(local_entity, remote_entity);
}

}