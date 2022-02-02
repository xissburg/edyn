#include "edyn/parallel/island_delta.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/collision/contact_manifold_map.hpp"
#include "edyn/config/config.h"
#include "edyn/util/vector.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

void island_delta::import_created_entities(entt::registry &registry, entity_map &map, bool mark_dirty) const {
    // Contact manifolds are a special case because it's necessary to check
    // whether there already is a manifold for the pair of bodies. Thus, created
    // entities which are manifolds, are handled separately where a new local
    // entity is only created if there isn't a manifold for that pair of bodies
    // yet in the contact manifold map.
    auto contact_manifold_id = entt::type_id<contact_manifold>().seq();
    using manifold_container_type = created_entity_component_container<contact_manifold>;
    const manifold_container_type *manifold_container = nullptr;

    if (m_created_components.count(contact_manifold_id)) {
        auto &created_ptr = m_created_components.at(contact_manifold_id);
        manifold_container = static_cast<const manifold_container_type *>(created_ptr.get());
    }

    for (auto remote_entity : m_created_entities) {
        if (map.has_rem(remote_entity)) continue;

        if (manifold_container) {
            auto found_it = std::find_if(manifold_container->pairs.begin(), manifold_container->pairs.end(),
                                         [&] (auto &&pair) { return pair.first == remote_entity; });
            if (found_it != manifold_container->pairs.end()) {
                continue;
            }
        }

        auto local_entity = registry.create();
        map.insert(remote_entity, local_entity);

        if (mark_dirty) {
            registry.get_or_emplace<dirty>(local_entity).set_new();
        }
    }

    if (manifold_container) {
        auto &manifold_map = registry.ctx<contact_manifold_map>();

        for (auto &pair : manifold_container->pairs) {
            auto remote_entity = pair.first;
            if (map.has_rem(remote_entity)) continue;

            auto local_body0 = map.remloc(pair.second.body[0]);
            auto local_body1 = map.remloc(pair.second.body[1]);
            entt::entity local_entity;

            if (manifold_map.contains(local_body0, local_body1)) {
                local_entity = manifold_map.get(local_body0, local_body1);

                if (map.has_loc(local_entity)) {
                    // Another remote entity maps into this local entity, which means
                    // a previous manifold existed between these bodies and was deleted
                    // in the source. Thus, just replace the old mapping with the new.
                    auto other_remote_entity = map.locrem(local_entity);
                    EDYN_ASSERT(vector_contains(m_destroyed_entities, other_remote_entity));
                    map.erase_rem(other_remote_entity);
                }
            } else {
                local_entity = registry.create();

                if (mark_dirty) {
                    registry.get_or_emplace<dirty>(local_entity).set_new();
                }
            }

            map.insert(remote_entity, local_entity);
        }
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

void island_delta::import_updated_components(entt::registry &registry, entity_map &map, bool mark_dirty) const {
    for (auto &pair : m_updated_components) {
        pair.second->import(registry, map, mark_dirty);
    }
}

void island_delta::import_created_components(entt::registry &registry, entity_map &map, bool mark_dirty) const {
    for (auto &pair : m_created_components) {
        pair.second->import(registry, map, mark_dirty);
    }
}

void island_delta::import_destroyed_components(entt::registry &registry, entity_map &map, bool mark_dirty) const {
    for (auto &pair : m_destroyed_components) {
        pair.second->import(registry, map, mark_dirty);
    }
}

void island_delta::import(entt::registry &registry, entity_map &map, bool mark_dirty) const {
    m_entity_map.each([&registry, &map] (entt::entity remote_entity, entt::entity local_entity) {
        if (!map.has_rem(remote_entity) && registry.valid(local_entity)) {
            map.insert(remote_entity, local_entity);
        }
    });

    import_created_entities(registry, map, mark_dirty);
    import_created_components(registry, map, mark_dirty);
    import_updated_components(registry, map, mark_dirty);
    import_destroyed_components(registry, map, mark_dirty);
    import_destroyed_entities(registry, map);
}

bool island_delta::empty() const {
    if (!m_entity_map.empty() ||
        !m_created_entities.empty() ||
        !m_destroyed_entities.empty()) {
        return false;
    }

    for (auto &pair : m_created_components) {
        if (!pair.second->empty()) {
            return false;
        }
    }

    for (auto &pair : m_updated_components) {
        if (!pair.second->empty()) {
            return false;
        }
    }

    for (auto &pair : m_destroyed_components) {
        if (!pair.second->empty()) {
            return false;
        }
    }

    return true;
}

}
