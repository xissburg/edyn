#include "edyn/parallel/island_delta.hpp"
#include <entt/entt.hpp>

namespace edyn {

void island_delta::import_created_entities(entt::registry &registry, entity_map &map) const {
    for (auto remote_entity : m_created_entities) {
        if (map.has_rem(remote_entity)) continue;
        auto local_entity = registry.create();
        map.insert(remote_entity, local_entity);
    }
}

void island_delta::import_destroyed_entities(entt::registry &registry, entity_map &map) const {
    for (auto remote_entity : m_destroyed_entities) {
        if (!map.has_rem(remote_entity)) continue;
        auto local_entity = map.remloc(remote_entity);
        map.erase_rem(remote_entity);

        if (registry.valid(local_entity)) {
            registry.destroy(local_entity);
        }
    }
}

void island_delta::import_updated_components(entt::registry &registry, entity_map &map) const {
    for (auto &ptr : m_updated_components) {
        if (!ptr) continue;
        ptr->import(*this, registry, map);
    }
}

void island_delta::import_created_components(entt::registry &registry, entity_map &map) const {
    for (auto &ptr : m_created_components) {
        if (!ptr) continue;
        ptr->import(*this, registry, map);
    }
}

void island_delta::import_destroyed_components(entt::registry &registry, entity_map &map) const {
    for (auto &ptr : m_destroyed_components) {
        if (!ptr) continue;
        ptr->import(*this, registry, map);
    }
}

void island_delta::import(entt::registry &registry, entity_map &map) const {
    m_entity_map.each([&registry, &map] (entt::entity remote_entity, entt::entity local_entity) {
        if (!map.has_rem(remote_entity) && registry.valid(local_entity)) {
            map.insert(remote_entity, local_entity);
        }
    });
    
    import_created_entities(registry, map);
    import_destroyed_entities(registry, map);

    import_created_components(registry, map);
    import_updated_components(registry, map);
    import_destroyed_components(registry, map);
}

}
