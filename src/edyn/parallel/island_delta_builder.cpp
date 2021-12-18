#include "edyn/parallel/island_delta_builder.hpp"
#include "edyn/comp/shared_comp.hpp"

namespace edyn {

void island_delta_builder::created(entt::entity entity) {
    m_delta.m_created_entities.push_back(entity);
}

void island_delta_builder::insert_entity_mapping(entt::entity remote_entity, entt::entity local_entity) {
    // Note that this is being called from the builder and the order is reversed,
    // i.e. (local, remote). When importing, the "correct" order is used, so the
    // first entity which is the remote, refers to the local entity in this registry.
    m_delta.m_entity_map.insert(local_entity, remote_entity);
}

bool island_delta_builder::has_rem(entt::entity remote_entity) const {
    // Remember that remote and local are swapped in the builder.
    return m_delta.m_entity_map.has_loc(remote_entity);
}

bool island_delta_builder::has_loc(entt::entity local_entity) const {
    // Remember that remote and local are swapped in the builder.
    return m_delta.m_entity_map.has_rem(local_entity);
}

bool island_delta_builder::empty() const {
    return m_delta.empty();
}

bool island_delta_builder::needs_wakeup() const {
    if (!m_delta.m_created_entities.empty() ||
        !m_delta.m_destroyed_entities.empty()) {
        return true;
    }

    for (auto &pair : m_delta.m_created_components) {
        if (!pair.second->empty()) {
            return false;
        }
    }

    for (auto &pair : m_delta.m_updated_components) {
        if (!pair.second->empty()) {
            return false;
        }
    }

    for (auto &pair : m_delta.m_destroyed_components) {
        if (!pair.second->empty()) {
            return false;
        }
    }

    return false;
}

}
