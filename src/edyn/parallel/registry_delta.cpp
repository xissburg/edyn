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

void registry_delta::import_updated_components(entt::registry &registry, entity_map &map) const {
    for (auto &pair : m_updated_components) {
        pair.second->import(*this, registry, map);
    }
}

void registry_delta::import_created_components(entt::registry &registry, entity_map &map) const {
    for (auto &pair : m_created_components) {
        pair.second->import(*this, registry, map);
    }
}

void registry_delta::import_destroyed_components(entt::registry &registry, entity_map &map) const {
    for (auto &pair : m_destroyed_components) {
        pair.second->import(*this, registry, map);
    }
}

void registry_delta::import(entt::registry &registry, entity_map &map) const {
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

bool registry_delta::empty() const {
    return 
        m_entity_map.empty() && 
        m_created_entities.empty() && 
        m_destroyed_entities.empty() &&
        m_created_components.empty() &&
        m_updated_components.empty() &&
        m_destroyed_components.empty();
}

}