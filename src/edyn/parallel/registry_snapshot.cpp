#include "edyn/parallel/registry_snapshot.hpp"
#include <entt/entt.hpp>

namespace edyn {

void registry_snapshot::import_created(entt::registry &registry, entity_map &map) const {
    for (auto remote_entity : m_created_entities) {
        if (map.has_rem(remote_entity)) continue;
        auto local_entity = registry.create();
        map.insert(remote_entity, local_entity);
    }
}

void registry_snapshot::import_child_entity_sequence(entt::registry &registry, entity_map &map, 
                                                     entt::meta_sequence_container *old_seq,
                                                     entt::meta_sequence_container &new_seq) const {
    if (new_seq.value_type() != entt::resolve<entt::entity>()) return;

    for (size_t i = 0; i < new_seq.size(); ++i) {
        auto &entity_ref = new_seq[i].cast<entt::entity>();

        if (entity_ref == entt::null) continue;

        if (map.has_rem(entity_ref)) {
            auto local_entity = map.remloc(entity_ref);
            if (registry.valid(local_entity)) {
                entity_ref = local_entity;
            } else {
                entity_ref = entt::null;
            }
        } else {
            entity_ref = entt::null;
        }
    }

    if (old_seq) {
        // Reinsert entities from old which are still valid and haven't been
        // moved to another island due to a split.
        for (auto old_it = old_seq->begin(); old_it != old_seq->end(); ++old_it) {
            const auto &old_entity = (*old_it).cast<entt::entity>();

            if (!registry.valid(old_entity)) continue;

            bool contains = false;
            for (auto it = new_seq.begin(); it != new_seq.end(); ++it) {
                auto &entity_ref = (*it).cast<entt::entity>();
                if (entity_ref == old_entity) {
                    contains = true;
                    break;
                }
            }
            if (contains) continue;

            bool moved = false;
            for (auto connected : m_split_connected_components) {
                for (auto remote_entity : connected) {
                    if (!map.has_rem(remote_entity)) continue;
                    auto local_entity = map.remloc(remote_entity);
                    if (!registry.valid(local_entity)) continue;
                    
                    if (local_entity == old_entity) {
                        moved = true;
                        break;
                    }
                }
            }
            if (moved) continue;

            auto result = new_seq.insert(new_seq.end(), old_entity);
            if (result.second) continue;

            // Not a dynamic container. Find a null entry to replace with.
            bool found = false;
            for (auto new_it = new_seq.begin(); new_it != new_seq.end(); ++new_it) {
                auto &new_entity = (*new_it).cast<entt::entity>();
                if (new_entity == entt::null) {
                    new_entity = old_entity;
                    found = true;
                    break;
                }
            }
            EDYN_ASSERT(found);
        }
    }
    
    // Move null entities to the end of the container.
    auto erase_it = new_seq.end();
    
    for (auto it = new_seq.begin(); it != new_seq.end(); ++it) {
        auto &entity_ref = (*it).cast<entt::entity>();
        if (entity_ref == entt::null) {
            erase_it = it;
            break;
        }
    }

    for (auto it = erase_it; it != new_seq.end(); ++it) {
        auto &entity_ref = (*it).cast<entt::entity>();
        if (entity_ref != entt::null) {
            (*erase_it).cast<entt::entity>() = entity_ref;
            ++erase_it;
        }
    }

    // Remove null entities if this is a dynamic container.
    for (auto it = erase_it; it != new_seq.end(); ) {
        auto result = new_seq.erase(it);

        if (!result.second) { // Not a dynamic container.
            break;
        }

        it = result.first;
    }
}

void registry_snapshot::import(entt::registry &registry, entity_map &map) const {
    import_created(registry, map);
    import_destroyed(registry, map, all_components{});

    for (auto remote_entity : m_destroyed_entities) {
        if (!map.has_rem(remote_entity)) continue;
        auto local_entity = map.remloc(remote_entity);
        map.erase_rem(remote_entity);

        if (registry.valid(local_entity)) {
            registry.destroy(local_entity);
        }
    }

    for (auto &pair : m_remloc_entity_pairs) {
        auto remote_entity = pair.first;
        auto local_entity = pair.second;

        if (!map.has_rem(remote_entity)) {
            if (local_entity != entt::null) {
                if (registry.valid(local_entity)) {
                    map.insert(remote_entity, local_entity);
                }
            }
        }
    }

    import_updated(registry, map, all_components{});
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