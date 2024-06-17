#ifndef EDYN_REPLICATION_ENTITY_MAP_HPP
#define EDYN_REPLICATION_ENTITY_MAP_HPP

#include "edyn/config/config.h"
#include <unordered_map>
#include <entt/entity/fwd.hpp>

namespace edyn {

/**
 * @brief Maps between remote and local entities and vice-versa.
 */
class entity_map {
public:
    /**
     * @brief Insert a new mapping.
     * @param entity Remote entity.
     * @param local Local entity.
     */
    void insert(entt::entity entity, entt::entity local) {
        EDYN_ASSERT(!map.count(entity));
        EDYN_ASSERT(!map_local.count(local));
        map[entity] = local;
        map_local[local] = entity;
    }

    /**
     * @brief Erase a remote entity.
     * @param entity Remote entity.
     */
    void erase(entt::entity entity) {
        auto local = map.at(entity);
        map.erase(entity);
        map_local.erase(local);
    }

    /**
     * @brief Erase a local entity.
     * @param local Local entity.
     */
    void erase_local(entt::entity local) {
        auto entity = map_local.at(local);
        map.erase(entity);
        map_local.erase(local);
    }

    /**
     * @brief Check if remote entity is present.
     * @param entity Remote entity.
     * @return Whether map contains given remote entity.
     */
    bool contains(entt::entity entity) const {
        return map.count(entity) > 0;
    }

    /**
     * @brief Check if local entity is present.
     * @param local Local entity.
     * @return Whether map contains given local entity.
     */
    bool contains_local(entt::entity local) const {
        return map_local.count(local) > 0;
    }

    /**
     * @brief Map remote entity to local.
     * @param entity Remote entity.
     * @return Corresponding local entity.
     */
    entt::entity at(entt::entity entity) const {
        return map.at(entity);
    }

    /**
     * @brief Map local entity to remote.
     * @param entity Local entity.
     * @return Corresponding remote entity.
     */
    entt::entity at_local(entt::entity local) const {
        return map_local.at(local);
    }

    /**
     * @brief Conditionally erase remote entities.
     * @tparam Predicate Function with signature `bool(entt::entity, entt::entity)`.
     * @param predicate Function which is called with a (remote, local) entity
     * pair and returns whether the mapping should be removed.
     */
    template<typename Predicate>
    void erase_if(Predicate predicate) {
        for (auto it = map.begin(); it != map.end();) {
            if (predicate(it->first, it->second)) {
                map_local.erase(it->second);
                it = map.erase(it);
            } else {
                ++it;
            }
        }
    }

    /**
     * @brief Iterate over each entity pair.
     * @tparam Func Function with signature `bool(entt::entity, entt::entity)`.
     * @param func Called with a (remote, local) entity pair.
     */
    template<typename Func>
    void each(Func func) const {
        for (auto it = map.begin(); it != map.end(); ++it) {
            func(it->first, it->second);
        }
    }

    /**
     * @brief Swaps remote and local entities.
     */
    void swap() {
        std::swap(map, map_local);
    }

    void clear() {
        map.clear();
        map_local.clear();
    }

private:
    std::unordered_map<entt::entity, entt::entity> map;
    std::unordered_map<entt::entity, entt::entity> map_local;
};

}

#endif // EDYN_REPLICATION_ENTITY_MAP_HPP
