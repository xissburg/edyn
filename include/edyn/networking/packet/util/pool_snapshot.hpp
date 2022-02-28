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
#include "edyn/parallel/merge/merge_component.hpp"
#include "edyn/parallel/merge/merge_contact_manifold.hpp"
#include "edyn/parallel/merge/merge_constraint.hpp"
#include "edyn/parallel/merge/merge_tree_view.hpp"
#include "edyn/parallel/merge/merge_collision_exclusion.hpp"
#include "edyn/parallel/merge/merge_entity_owner.hpp"
#include "edyn/parallel/entity_component_container.hpp"

namespace edyn {

struct pool_snapshot_data {
    virtual std::vector<entt::entity> get_entities() const = 0;
    virtual void convert_remloc(entity_map &) = 0;
    virtual void write(memory_output_archive &archive) = 0;
    virtual void read(memory_input_archive &archive) = 0;
    virtual void replace_into_registry(entt::registry &registry, const entity_map &emap) = 0;
    virtual bool empty() const = 0;
    virtual entt::id_type get_type_id() const = 0;
};

template<typename Component>
struct pool_snapshot_data_impl : public pool_snapshot_data {
    static constexpr auto is_empty_type = std::is_empty_v<Component>;
    using element_type = std::conditional_t<is_empty_type, entt::entity, std::pair<entt::entity, Component>>;
    std::vector<element_type> data;

    std::vector<entt::entity> get_entities() const override {
        if constexpr(is_empty_type) {
            return data;
        } else {
            auto entities = std::vector<entt::entity>(data.size());
            std::transform(data.begin(), data.end(), entities.begin(), [] (auto &&pair) { return pair.first; });
            return entities;
        }
    }

    void convert_remloc(entity_map &emap) override {
        if constexpr(is_empty_type) {
            for (auto &entity : data) {
                entity = emap.remloc(entity);
            }
        } else {
            for (auto &pair : data) {
                pair.first = emap.remloc(pair.first);
                // `merge` maps remote to local.
                merge(pair.second, emap);
            }
        }
    }

    void write(memory_output_archive &archive) override {
        archive(data);
    }

    void read(memory_input_archive &archive) override {
        archive(data);
    }

    void replace_into_registry(entt::registry &registry, const entity_map &emap) override {
        if constexpr(!is_empty_type) {
            for (auto &pair : data) {
                auto remote_entity = pair.first;
                if (!emap.has_rem(remote_entity)) continue;
                auto local_entity = emap.remloc(pair.first);
                if (!registry.valid(local_entity)) continue;
                if (!registry.all_of<Component>(local_entity)) continue;
                auto &comp = pair.second;
                merge(comp, emap);
                registry.replace<Component>(local_entity, comp);
            }
        }
    }

    void insert(const entt::registry &registry, entt::entity entity) {
        if constexpr(is_empty_type) {
            data.push_back(entity);
        } else {
            auto &comp = registry.get<Component>(entity);
            data.push_back(std::make_pair(entity, comp));
        }
    }

    bool empty() const override {
        return data.empty();
    }

    entt::id_type get_type_id() const override {
        return entt::type_id<Component>().seq();
    }
};

struct pool_snapshot {
    unsigned component_index;
    std::shared_ptr<pool_snapshot_data> ptr;
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
    template<typename Comp>
    void pool_insert_entity_component_single(const entt::registry &registry, entt::entity entity,
                                             std::vector<pool_snapshot> &pools, unsigned component_index) {
        if (!registry.any_of<Comp>(entity)) {
            return;
        }

        using pool_snapshot_data_t = pool_snapshot_data_impl<Comp>;

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

        auto typed_pool = std::static_pointer_cast<pool_snapshot_data_t>(pool->ptr);
        typed_pool->insert(registry, entity);
    }

    template<typename... Components, typename IndexType, IndexType... Is>
    void pool_insert_entity_components_all(const entt::registry &registry, entt::entity entity,
                                           std::vector<pool_snapshot> &pools,
                                           [[maybe_unused]] std::tuple<Components...>,
                                           [[maybe_unused]] std::integer_sequence<IndexType, Is...>) {
        (pool_insert_entity_component_single<Components>(registry, entity, pools, Is), ...);
    }

    template<typename Component, typename... Components>
    void pool_insert_select_entity_component(const entt::registry &registry, entt::entity entity,
                                             std::vector<pool_snapshot> &pools,
                                             [[maybe_unused]] std::tuple<Components...>) {
        constexpr auto index = index_of_v<size_t, Component, Components...>;
        pool_insert_entity_component_single<Component>(registry, entity, pools, index);
    }

    template<typename... SelectComponents, typename... Components>
    void pool_insert_select_entity_components(const entt::registry &registry, entt::entity entity,
                                              std::vector<pool_snapshot> &pools,
                                              std::tuple<Components...> components) {
        (pool_insert_select_entity_component<SelectComponents>(registry, entity, pools, components), ...);
    }
}

}

#endif // EDYN_NETWORKING_PACKET_POOL_SNAPSHOT_HPP
