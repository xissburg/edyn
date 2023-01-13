#ifndef EDYN_NETWORKING_UTIL_REGISTRY_SNAPSHOT_HPP
#define EDYN_NETWORKING_UTIL_REGISTRY_SNAPSHOT_HPP

#include <vector>
#include "edyn/replication/entity_map.hpp"
#include "edyn/util/tuple_util.hpp"
#include "edyn/networking/util/pool_snapshot.hpp"

namespace edyn::packet {

/**
 * @brief Fundamental data structure for all networked data transfer containing
 * entities and components. It contains an array of entities and an array of
 * component pools where each pool holds an array of entity indices referring to
 * the array of entities and, if the component type is not empty, it also
 * contains an array of components in a 1-to-1 relationship with the entity
 * index array.
 */
struct registry_snapshot {
    double timestamp;
    std::vector<entt::entity> entities;
    std::vector<pool_snapshot> pools;

    void convert_remloc(const entt::registry &registry, const entity_map &emap) {
        for (auto &entity : entities) {
            entity = emap.at(entity);
        }

        for (auto &pool : pools) {
            pool.ptr->convert_remloc(registry, emap);
        }
    }
};

template<typename Archive>
void serialize(Archive &archive, registry_snapshot &snapshot) {
    archive(snapshot.timestamp);
    archive(snapshot.entities);
    archive(snapshot.pools);
}

}

// Registry snapshot utility functions.
namespace edyn::internal {
    template<typename Component>
    pool_snapshot_data_impl<Component> * get_pool(std::vector<pool_snapshot> &pools, component_index_type component_index) {
        using pool_snapshot_data_t = pool_snapshot_data_impl<Component>;

        auto pool = std::find_if(pools.begin(), pools.end(),
                                 [component_index](auto &&pool) {
                                     return pool.component_index == component_index;
                                 });

        if (pool == pools.end()) {
            pools.push_back(pool_snapshot{component_index});
            pool = pools.end();
            std::advance(pool, -1);
            pool->ptr.reset(new pool_snapshot_data_t);
        }

        auto *typed_pool = static_cast<pool_snapshot_data_t *>(pool->ptr.get());
        return typed_pool;
    }

    template<typename Component>
    void snapshot_insert_entity(const entt::registry &registry, entt::entity entity,
                                packet::registry_snapshot &snap, component_index_type component_index) {
        get_pool<Component>(snap.pools, component_index)->insert_single(registry, entity, snap.entities);
    }

    template<typename Component, typename It>
    void snapshot_insert_entities(const entt::registry &registry, It first, It last,
                                  packet::registry_snapshot &snap, component_index_type component_index) {
        get_pool<Component>(snap.pools, component_index)->insert(registry, first, last, snap.entities);
    }
}

#endif // EDYN_NETWORKING_UTIL_REGISTRY_SNAPSHOT_HPP
