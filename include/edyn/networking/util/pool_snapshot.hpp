#ifndef EDYN_NETWORKING_PACKET_POOL_SNAPSHOT_HPP
#define EDYN_NETWORKING_PACKET_POOL_SNAPSHOT_HPP

#include <algorithm>
#include <array>
#include <iterator>
#include <memory>
#include <tuple>
#include <vector>
#include <utility>
#include <entt/entity/fwd.hpp>
#include "edyn/networking/comp/networked_comp.hpp"
#include "edyn/serialization/memory_archive.hpp"
#include "edyn/util/entity_map.hpp"
#include "edyn/util/tuple_util.hpp"
#include "edyn/parallel/map_child_entity.hpp"

namespace edyn {

struct pool_snapshot_data {
    using index_type = uint8_t;
    std::vector<index_type> entity_indices;

    virtual ~pool_snapshot_data() = default;
    virtual void convert_remloc(const entt::registry &registry, const entity_map &emap) = 0;
    virtual void write(memory_output_archive &archive) = 0;
    virtual void read(memory_input_archive &archive) = 0;
    virtual void replace_into_registry(entt::registry &registry,
                                       const std::vector<entt::entity> &entities,
                                       const entity_map &emap) = 0;
    virtual entt::id_type get_type_id() const = 0;

    bool empty() const {
        return entity_indices.empty();
    }
};

template<typename Component>
struct pool_snapshot_data_impl : public pool_snapshot_data {
    static constexpr auto is_empty_type = std::is_empty_v<Component>;
    std::vector<Component> components;

    void convert_remloc(const entt::registry &registry, const entity_map &emap) override {
        if constexpr(!is_empty_type) {
            for (auto &comp : components) {
                internal::map_child_entity(registry, emap, comp);
            }
        }
    }

    void write(memory_output_archive &archive) override {
        index_type num_entities = static_cast<index_type>(entity_indices.size());
        archive(num_entities);

        for (auto &idx : entity_indices) {
            archive(idx);
        }

        if constexpr(!is_empty_type) {
            for (auto &comp : components) {
                archive(comp);
            }
        }
    }

    void read(memory_input_archive &archive) override {
        index_type num_entities;
        archive(num_entities);
        entity_indices.resize(num_entities);

        for (auto &idx : entity_indices) {
            archive(idx);
        }

        if constexpr(!is_empty_type) {
            components.resize(num_entities);

            for (auto &comp : components) {
                archive(comp);
            }
        }
    }

    void replace_into_registry(entt::registry &registry,
                               const std::vector<entt::entity> &pool_entities,
                               const entity_map &emap) override {
        if constexpr(!is_empty_type) {
            EDYN_ASSERT(entity_indices.size() == components.size());

            for (size_t i = 0; i < entity_indices.size(); ++i) {
                auto entity_index = entity_indices[i];
                auto remote_entity = pool_entities[entity_index];
                if (!emap.contains(remote_entity)) continue;
                auto local_entity = emap.at(remote_entity);
                if (!registry.valid(local_entity)) continue;
                if (!registry.all_of<Component>(local_entity)) continue;
                auto &comp = components[i];
                internal::map_child_entity(registry, emap, comp);
                registry.replace<Component>(local_entity, comp);
            }
        }
    }

    void insert_all(const entt::registry &registry,
                    const std::vector<entt::entity> &pool_entities) {
        auto view = registry.view<Component>();

        for (index_type idx = 0; idx < pool_entities.size(); ++idx) {
            auto entity = pool_entities[idx];

            if (view.template contains(entity)) {
                entity_indices.push_back(idx);

                if constexpr(!is_empty_type) {
                    auto [comp] = view.get(entity);
                    components.push_back(comp);
                }
            }
        }
    }

    void insert_single(const entt::registry &registry,
                       const std::vector<entt::entity> &pool_entities,
                       entt::entity entity) {
        auto found_it = std::find(pool_entities.begin(), pool_entities.end(), entity);

        if (found_it == pool_entities.end()) {
            return;
        }

        auto view = registry.view<Component>();

        if (!view.contains(entity)) {
            return;
        }

        auto idx = std::distance(pool_entities.begin(), found_it);
        entity_indices.push_back(idx);

        if constexpr(!is_empty_type) {
            auto [comp] = view.get(entity);
            components.push_back(comp);
        }
    }

    template<typename It>
    void insert(const entt::registry &registry,
                const std::vector<entt::entity> &pool_entities,
                It first, It last) {
        auto view = registry.view<Component>();

        for (; first != last; ++first) {
            auto entity = *first;
            auto found_it = std::find(pool_entities.begin(), pool_entities.end(), entity);

            if (!view.contains(entity) || found_it == pool_entities.end()) {
                continue;
            }

            auto idx = std::distance(pool_entities.begin(), found_it);
            entity_indices.push_back(idx);

            if constexpr(!is_empty_type) {
                auto [comp] = view.get(entity);
                components.push_back(comp);
            }
        }
    }

    entt::id_type get_type_id() const override {
        return entt::type_id<Component>().seq();
    }
};

struct pool_snapshot {
    unsigned component_index;
    std::shared_ptr<pool_snapshot_data> ptr;
};

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

template<typename... Components>
auto create_make_pool_snapshot_data_function([[maybe_unused]] std::tuple<Components...>) {
    return [] (unsigned component_index) {
        std::tuple<Components...> components;
        auto ptr = std::unique_ptr<pool_snapshot_data>{};
        visit_tuple(components, component_index, [&] (auto &&c) {
            using CompType = std::decay_t<decltype(c)>;
            ptr.reset(new pool_snapshot_data_impl<CompType>);
        });
        return ptr;
    };
}

extern std::unique_ptr<pool_snapshot_data>(*g_make_pool_snapshot_data)(unsigned);

template<typename Archive>
void serialize(Archive &archive, pool_snapshot &pool) {
    archive(pool.component_index);
    std::vector<uint8_t> data;

    if constexpr(Archive::is_input::value) {
        archive(data);
        auto input = memory_input_archive(data.data(), data.size());
        pool.ptr = (*g_make_pool_snapshot_data)(pool.component_index);
        pool.ptr->read(input);
    } else {
        auto output = memory_output_archive(data);
        pool.ptr->write(output);
        archive(data);
    }
}

// Pool snapshot utility functions.
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
    void pool_insert_entities(const entt::registry &registry,
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
    void pool_insert_entity(const entt::registry &registry, entt::entity entity,
                            registry_snapshot &snap, unsigned component_index) {
        auto found_it = std::find(snap.entities.begin(), snap.entities.end(), entity);

        if (found_it != snap.entities.end() && registry.all_of<Component>(entity)) {
            get_pool<Component>(snap.pools, component_index)->insert_single(registry, snap.entities, entity);
        }
    }

    template<typename... Components, typename IndexType, IndexType... Is>
    void pool_insert_entity_components_all(const entt::registry &registry,
                                           registry_snapshot &snap,
                                           [[maybe_unused]] std::tuple<Components...>,
                                           [[maybe_unused]] std::integer_sequence<IndexType, Is...>) {
        (pool_insert_entities<Components>(registry, snap, Is), ...);
    }

    template<typename Component, typename... Components>
    void pool_insert_select_entity_component(const entt::registry &registry,
                                             registry_snapshot &snap,
                                             [[maybe_unused]] std::tuple<Components...>) {
        constexpr auto index = index_of_v<size_t, Component, Components...>;
        pool_insert_entities<Component>(registry, snap, index);
    }

    template<typename... SelectComponents, typename... Components>
    void pool_insert_select_entity_components(const entt::registry &registry,
                                              registry_snapshot &snap,
                                              std::tuple<Components...> components) {
        (pool_insert_select_entity_component<SelectComponents>(registry, snap, components), ...);
    }
}

}

#endif // EDYN_NETWORKING_PACKET_POOL_SNAPSHOT_HPP
