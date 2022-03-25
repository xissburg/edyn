#ifndef EDYN_NETWORKING_UTIL_REGISTRY_SNAPSHOT_HPP
#define EDYN_NETWORKING_UTIL_REGISTRY_SNAPSHOT_HPP

#include <vector>
#include "edyn/util/entity_map.hpp"
#include "edyn/util/tuple_util.hpp"
#include "edyn/networking/util/pool_snapshot.hpp"

namespace edyn {

/**
 * @brief Fundamental data structure for all networked data transfer containing
 * entities and components. It contains an array of entities and an array of
 * component pools where each pool holds an array of entity indices referring to
 * the array of entities and, if the component type is not empty, it also
 * contains an array of components in a 1-to-1 relationship with the entity
 * index array.
 */
struct registry_snapshot {
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

// Registry snapshot utility functions.
namespace internal {
    template<typename Component>
    pool_snapshot_data_impl<Component> * get_pool(std::vector<pool_snapshot> &pools, unsigned component_index) {
        using pool_snapshot_data_t = pool_snapshot_data_impl<Component>;

        auto pool = std::find_if(pools.begin(), pools.end(),
                                 [component_index] (auto &&pool) {
                                     return pool.component_index == component_index;
                                 });

        if (pool == pools.end()) {
            pools.push_back(pool_snapshot{unsigned(component_index)});
            pool = pools.end();
            std::advance(pool, -1);
            pool->ptr.reset(new pool_snapshot_data_t);
        }

        auto *typed_pool = static_cast<pool_snapshot_data_t *>(pool->ptr.get());
        return typed_pool;
    }

    template<typename Component>
    void snapshot_insert_all(const entt::registry &registry,
                             registry_snapshot &snap, unsigned component_index) {
        auto view = registry.view<Component>();
        auto entity_has_component =
            std::find_if(
                snap.entities.begin(), snap.entities.end(),
                [&] (auto entity) {
                    return view.contains(entity);
                }) != snap.entities.end();

        if (entity_has_component) {
            get_pool<Component>(snap.pools, component_index)->insert_all(registry, snap.entities);
        }
    }

    template<typename Component>
    void snapshot_insert_entity(const entt::registry &registry, entt::entity entity,
                                registry_snapshot &snap, unsigned component_index) {
        auto found_it = std::find(snap.entities.begin(), snap.entities.end(), entity);

        if (found_it != snap.entities.end()) {
            get_pool<Component>(snap.pools, component_index)->insert_single(registry, entity, snap.entities);
        }
    }

    template<typename Component, typename It>
    void snapshot_insert_entities(const entt::registry &registry, It first, It last,
                                  registry_snapshot &snap, unsigned component_index) {
        get_pool<Component>(snap.pools, component_index)->insert(registry, first, last, snap.entities);
    }

    template<typename... Components, typename IndexType, IndexType... Is>
    void snapshot_insert_entity_components_all(const entt::registry &registry,
                                               registry_snapshot &snap,
                                               [[maybe_unused]] std::tuple<Components...>,
                                               [[maybe_unused]] std::integer_sequence<IndexType, Is...>) {
        (snapshot_insert_all<Components>(registry, snap, Is), ...);
    }

    template<typename Component, typename... Components>
    void snapshot_insert_select_entity_component(const entt::registry &registry,
                                                 registry_snapshot &snap,
                                                 [[maybe_unused]] std::tuple<Components...>) {
        constexpr auto index = index_of_v<size_t, Component, Components...>;
        snapshot_insert_all<Component>(registry, snap, index);
    }

    template<typename... SelectComponents, typename... Components>
    void snapshot_insert_select_entity_components(const entt::registry &registry,
                                                  registry_snapshot &snap,
                                                  std::tuple<Components...> components) {
        (snapshot_insert_select_entity_component<SelectComponents>(registry, snap, components), ...);
    }
}

}

#endif // EDYN_NETWORKING_UTIL_REGISTRY_SNAPSHOT_HPP
