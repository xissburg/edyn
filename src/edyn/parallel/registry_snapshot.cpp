#include "edyn/parallel/registry_snapshot.hpp"
#include <entt/entt.hpp>

namespace edyn {

void registry_snapshot::import_child_entity_sequence(entt::registry &registry, const entity_map &map, entt::meta_sequence_container &seq) const {
    if (seq.value_type() != entt::resolve<entt::entity>()) return;

    // HACK: there's currently no other way to determine whether
    // this is a dynamic or fixed container (i.e. std::vector or
    // std::array). `resize` will return false if it's fixed.
    auto is_dynamic = seq.resize(seq.size());

    if (is_dynamic) {
        // Only include valid entities.
        std::vector<entt::entity> entities;
        entities.reserve(seq.size());

        for (size_t i = 0; i < seq.size(); ++i) {
            auto remote_entity = seq[i].cast<entt::entity>();
            if (map.has_rem(remote_entity)) {
                auto local_entity = map.remloc(remote_entity);
                if (registry.valid(local_entity)) {
                    entities.push_back(local_entity);
                }
            }
        }

        seq.clear();
        for (auto entity : entities) {
            seq.insert(seq.end(), entity);
        }
    } else {
        for (size_t i = 0; i < seq.size(); ++i) {
            auto remote_entity = seq[i].cast<entt::entity>();
            if (map.has_rem(remote_entity)) {
                auto local_entity = map.remloc(remote_entity);
                if (registry.valid(local_entity)) {
                    seq[i].cast<entt::entity>() = local_entity;
                } else {
                    seq[i].cast<entt::entity>() = entt::null;
                }
            } else {
                seq[i].cast<entt::entity>() = entt::null;
            }
        }
    }
}

void registry_snapshot::import(entt::registry &registry, entity_map &map) const {
    for (auto &pair : m_remloc_entity_pairs) {
        auto remote_entity = pair.first;
        auto local_entity = pair.second;

        if (!map.has_rem(remote_entity)) {
            if (local_entity != entt::null) {
                map.insert(remote_entity, local_entity);
            } else {
                map.insert(remote_entity, registry.create());
            }
        }
    }

    import_updated(registry, map, all_components{});
    import_destroyed(registry, map, all_components{});

    for (auto remote_entity : m_destroyed_entities) {
        if (!map.has_rem(remote_entity)) continue;
        auto local_entity = map.remloc(remote_entity);
        map.erase_rem(remote_entity);

        if (registry.valid(local_entity)) {
            registry.destroy(local_entity);
        }
    }
}

void registry_snapshot_builder::insert_entity_mapping(entt::entity local_entity) {
    // Ignore if mapping already exists. Note that this is being called from the
    // builder and the order is reversed, i.e. (local, remote). When importing, 
    // the default order is used, so the first entity which is the remote, refers
    // to the local entity in this registry.
    auto found_it = std::find_if(
        m_snapshot.m_remloc_entity_pairs.begin(), 
        m_snapshot.m_remloc_entity_pairs.end(), 
        [local_entity] (auto &pair) { return local_entity == pair.first; });

    if (found_it != m_snapshot.m_remloc_entity_pairs.end()) {
        return;
    }
    
    if (m_entity_map->has_loc(local_entity)) {
        auto remote_entity = m_entity_map->locrem(local_entity);
        m_snapshot.m_remloc_entity_pairs.emplace_back(local_entity, remote_entity);
    } else {
        m_snapshot.m_remloc_entity_pairs.emplace_back(local_entity, entt::null);
    }
}

}