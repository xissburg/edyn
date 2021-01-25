#include "edyn/parallel/registry_delta_builder.hpp"
#include "edyn/comp/shared_comp.hpp"

namespace edyn {

std::unique_ptr<registry_delta_builder> make_registry_delta_builder_default(entity_map &map) {
    return std::unique_ptr<registry_delta_builder>(
        new registry_delta_builder_impl(map, shared_components{}));
}

make_registry_delta_builder_func_t g_make_registry_delta_builder = &make_registry_delta_builder_default;

std::unique_ptr<registry_delta_builder> make_registry_delta_builder(entity_map &map) {
    return (*g_make_registry_delta_builder)(map);
}

void remove_external_components() {
    g_make_registry_delta_builder = &make_registry_delta_builder_default;
}

void registry_delta_builder::created(entt::entity entity) {
    m_delta.m_created_entities.push_back(entity);
}

void registry_delta_builder::insert_entity_mapping(entt::entity local_entity) {
    // Note that this is being called from the builder and the order is reversed,
    // i.e. (local, remote). When importing, the "correct" order is used, so the
    // first entity which is the remote, refers to the local entity in this registry.
    auto remote_entity = m_entity_map->locrem(local_entity);
    m_delta.m_entity_map.insert(local_entity, remote_entity);
}

bool registry_delta_builder::empty() const {
    if (!m_delta.m_entity_map.empty() || 
        !m_delta.m_created_entities.empty() || 
        !m_delta.m_destroyed_entities.empty()) {
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
