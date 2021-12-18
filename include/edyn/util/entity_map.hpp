#ifndef EDYN_UTIL_ENTITY_MAP_HPP
#define EDYN_UTIL_ENTITY_MAP_HPP

#include <unordered_map>
#include <entt/entity/fwd.hpp>
#include "edyn/config/config.h"

namespace edyn {

/**
 * Maps entities from one registry to another and back.
 */
class entity_map {
public:
    void insert(entt::entity remote_entity, entt::entity local_entity) {
        EDYN_ASSERT(!has_rem(remote_entity));
        EDYN_ASSERT(!has_loc(local_entity));
        m_remloc[remote_entity] = local_entity;
        m_locrem[local_entity] = remote_entity;
    }

    bool has_rem(entt::entity remote_entity) const {
        return m_remloc.count(remote_entity);
    }

    bool has_loc(entt::entity local_entity) const {
        return m_locrem.count(local_entity);
    }

    entt::entity remloc(entt::entity remote_entity) const {
        EDYN_ASSERT(has_rem(remote_entity));
        return m_remloc.at(remote_entity);
    }

    entt::entity locrem(entt::entity local_entity) const {
        EDYN_ASSERT(has_loc(local_entity));
        return m_locrem.at(local_entity);
    }

    void erase_rem(entt::entity remote_entity) {
        auto local_entity = remloc(remote_entity);
        m_remloc.erase(remote_entity);
        m_locrem.erase(local_entity);
    }

    void erase_loc(entt::entity local_entity) {
        auto remote_entity = locrem(local_entity);
        m_remloc.erase(remote_entity);
        m_locrem.erase(local_entity);
    }

    bool empty() const {
        return m_remloc.empty();
    }

    void clear() {
        m_remloc.clear();
        m_locrem.clear();
    }

    template<typename Func>
    void each(Func func) const {
        for (auto &pair : m_remloc) {
            func(pair.first, pair.second);
        }
    }

private:
    std::unordered_map<entt::entity, entt::entity> m_remloc; // Maps remote to local entities.
    std::unordered_map<entt::entity, entt::entity> m_locrem; // Maps local to remote entities.
};

}

#endif // EDYN_UTIL_ENTITY_MAP_HPP
