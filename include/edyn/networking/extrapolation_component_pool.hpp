#ifndef EDYN_NETWORKING_EXTRAPOLATION_COMPONENT_POOL_HPP
#define EDYN_NETWORKING_EXTRAPOLATION_COMPONENT_POOL_HPP

#include "edyn/util/entity_map.hpp"
#include <entt/entity/registry.hpp>
#include <type_traits>

namespace edyn {

struct extrapolation_component_pool {
    virtual void emplace(entt::registry &, entity_map &) = 0;
    virtual void replace(entt::registry &, entity_map &) = 0;
    virtual void convert_locrem(entity_map &) = 0;
};

template<typename Component>
struct extrapolation_component_pool_impl : public extrapolation_component_pool {
    static constexpr auto is_empty_type = std::is_empty_v<Component>;
    using element_type = std::conditional_t<is_empty_type, entt::entity, std::pair<entt::entity, Component>>;
    std::vector<element_type> data;

    void emplace(entt::registry &registry, entity_map &emap) override {
        if constexpr(is_empty_type) {
            for (auto remote_entity : data) {
                auto local_entity = emap.remloc(remote_entity);
                registry.emplace<Component>(local_entity);
            }
        } else {
            for (auto &pair : data) {
                auto remote_entity = pair.first;
                auto local_entity = emap.remloc(remote_entity);
                auto &comp = pair.second;
                merge(comp, emap);
                registry.emplace<Component>(local_entity, std::move(comp));
            }
        }
    }

    void replace(entt::registry &registry, entity_map &emap) override {
        if constexpr(!is_empty_type) {
            for (auto &pair : data) {
                auto remote_entity = pair.first;
                if (!emap.has_rem(remote_entity)) continue;
                auto local_entity = emap.remloc(remote_entity);
                if (!registry.valid(local_entity)) continue;
                if (!registry.all_of<Component>(local_entity)) continue;
                auto &comp = pair.second;
                merge(comp, emap);
                registry.replace<Component>(local_entity, std::move(comp));
            }
        }
    }

    void convert_locrem(entity_map &emap) override {
        if constexpr(is_empty_type) {
            for (auto &entity : data) {
                entity = emap.locrem(entity);
            }
        } else {
            for (auto &pair : data) {
                pair.first = emap.locrem(pair.first);
            }

            // `merge` maps remote to local, thus flip the entity map.
            emap.flip();
            for (auto &pair : data) {
                merge(pair.second, emap);
            }
            emap.flip();
        }
    }
};

namespace internal {
    using extrapolation_component_pool_vector = std::vector<std::shared_ptr<extrapolation_component_pool>>;

    template<typename Component>
    void extrapolation_component_pools_import_single(extrapolation_component_pool_vector &pools,
                                                     const entt::registry &registry,
                                                     const entt::sparse_set &entities) {
        auto view = registry.view<Component>();

        if (view.empty()) {
            return;
        }

        // Check if view contains any of the given entities.
        auto any_it = std::find_if(entities.begin(), entities.end(),
                                   [view] (auto entity) { return view.contains(entity); });

        if (any_it == entities.end()) {
            return;
        }

        auto pool = std::make_unique<extrapolation_component_pool_impl<Component>>();
        pool->data.reserve(entities.size());

        for (auto entity : entities) {
            if (view.contains(entity)) {
                if constexpr(std::is_empty_v<Component>) {
                    pool->data.push_back(entity);
                } else {
                    auto [comp] = view.template get(entity);
                    pool->data.emplace_back(entity, comp);
                }
            }
        }

        pools.push_back(std::move(pool));
    }

    // Import entities from main registry to be extrapolated.
    template<typename... Component>
    void extrapolation_component_pools_import_all(extrapolation_component_pool_vector &pools,
                                                  const entt::registry &registry,
                                                  const entt::sparse_set &entities) {
        (extrapolation_component_pools_import_single<Component>(pools, registry, entities), ...);
    }

    template<typename... Component>
    void extrapolation_component_pools_import_by_id(extrapolation_component_pool_vector &pools,
                                                     const entt::registry &registry,
                                                     const entt::sparse_set &entities, entt::id_type id) {
        ((entt::type_id<Component>().seq() == id ?
            extrapolation_component_pools_import_single<Component>(pools, registry, entities) : void(0)), ...);
    }

    template<typename... Component>
    auto make_extrapolation_component_pools_import_func([[maybe_unused]] std::tuple<Component...>) {
        return [] (extrapolation_component_pool_vector &pools,
                   const entt::registry &registry, const entt::sparse_set &entities) {
            extrapolation_component_pools_import_all<Component...>(pools, registry, entities);
        };
    }

    template<typename... Component>
    auto make_extrapolation_component_pools_import_by_id_func([[maybe_unused]] std::tuple<Component...>) {
        return [] (extrapolation_component_pool_vector &pools,
                   const entt::registry &registry,
                   const entt::sparse_set &entities, entt::id_type id) {
            extrapolation_component_pools_import_by_id<Component...>(pools, registry, entities, id);
        };
    }
}

}

#endif // EDYN_NETWORKING_EXTRAPOLATION_COMPONENT_POOL_HPP
