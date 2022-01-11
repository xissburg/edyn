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
#include "edyn/networking/comp/transient_comp.hpp"
#include "edyn/serialization/memory_archive.hpp"
#include "edyn/util/entity_map.hpp"
#include "edyn/util/tuple_util.hpp"
#include "edyn/parallel/merge/merge_component.hpp"
#include "edyn/parallel/merge/merge_contact_manifold.hpp"
#include "edyn/parallel/merge/merge_constraint.hpp"
#include "edyn/parallel/merge/merge_tree_view.hpp"
#include "edyn/parallel/merge/merge_collision_exclusion.hpp"

namespace edyn {

struct pool_snapshot_data_base {
    virtual std::vector<entt::entity> get_entities() const = 0;
    virtual void convert_remloc(entity_map &) = 0;
};

template<typename Component>
struct pool_snapshot_data : public pool_snapshot_data_base {
    std::vector<std::pair<entt::entity, Component>> pairs;

    std::vector<entt::entity> get_entities() const override {
        auto entities = std::vector<entt::entity>(pairs.size());
        std::transform(pairs.begin(), pairs.end(), entities.begin(), [] (auto &&pair) { return pair.first; });
        return entities;
    }

    void convert_remloc(entity_map &emap) override {
        for (auto &pair : pairs) {
            pair.first = emap.remloc(pair.first);
            merge(pair.second, emap);
        }
    }
};

struct pool_snapshot {
    unsigned component_index;
    std::shared_ptr<pool_snapshot_data_base> ptr;
};

template<typename Archive, typename Component>
void serialize(Archive &archive, pool_snapshot_data<Component> &pool) {
    archive(pool.pairs);
}

namespace detail {
    template<typename Archive, typename Component>
    constexpr auto make_serialize_pool_snapshot_base_function() {
        return [] (Archive &archive, pool_snapshot_data_base &pool) {
            auto &typed_pool = static_cast<pool_snapshot_data<Component> &>(pool);
            serialize(archive, typed_pool);
        };
    }

    template<typename Archive, typename... Component>
    struct serialize_pool_snapshot_base_function_array {
        using FunctionType = void(*)(Archive &, pool_snapshot_data_base &);
        std::array<FunctionType, sizeof...(Component)> functions;

        constexpr serialize_pool_snapshot_base_function_array()
            : functions{}
        {
            unsigned i = 0;
            ((functions[i++] = make_serialize_pool_snapshot_base_function<Archive, Component>()), ...);
        }
    };

    template<typename Archive, typename... Component>
    void serialize(Archive &archive, unsigned component_index, pool_snapshot_data_base &pool, std::tuple<Component...>) {
        static constexpr auto func_array = detail::serialize_pool_snapshot_base_function_array<Archive, Component...>();
        func_array.functions[component_index](archive, pool);
    }
}

class pool_snapshot_serializer_base {
public:
    virtual void write(std::vector<uint8_t> &, const pool_snapshot &) const = 0;
    virtual void read(const std::vector<uint8_t> &, pool_snapshot &) const = 0;
};

class pool_snapshot_serializer {
public:
    std::shared_ptr<pool_snapshot_serializer_base> ptr;
};

template<typename... Component>
class pool_snapshot_serializer_impl : public pool_snapshot_serializer_base {
public:
    pool_snapshot_serializer_impl(std::tuple<Component...>) {}

    std::tuple<Component...> components;

    void write(std::vector<uint8_t> &data, const pool_snapshot &pool) const override {
        auto archive = memory_output_archive(data);
        detail::serialize(archive, pool.component_index, *pool.ptr, components);
    }

    virtual void read(const std::vector<uint8_t> &data, pool_snapshot &pool) const override {
        visit_tuple(components, pool.component_index, [&] (auto &&c) {
            using CompType = std::decay_t<decltype(c)>;
            pool.ptr.reset(new pool_snapshot_data<CompType>);
        });

        auto archive = memory_input_archive(data.data(), data.size());
        detail::serialize(archive, pool.component_index, *pool.ptr, components);
    }
};

static pool_snapshot_serializer g_pool_snapshot_serializer = [] () {
    auto s = pool_snapshot_serializer{};
    s.ptr.reset(new pool_snapshot_serializer_impl(networked_components));
    return s;
}();

template<typename Archive>
void serialize(Archive &archive, pool_snapshot &pool) {
    archive(pool.component_index);

    if constexpr(Archive::is_input::value) {
        std::vector<uint8_t> data;
        archive(data);
        g_pool_snapshot_serializer.ptr->read(data, pool);
    } else {
        std::vector<uint8_t> data;
        g_pool_snapshot_serializer.ptr->write(data, pool);
        archive(data);
    }
}

template<typename Component>
void insert_entity_component(entt::registry &registry, entt::entity entity, std::vector<pool_snapshot> &pools, unsigned index) {
    if (!registry.any_of<Component>(entity)) {
        return;
    }

    auto pool = std::find_if(pools.begin(), pools.end(), [index] (auto &&pool) { return pool.component_index == index; });

    if (pool == pools.end()) {
        pools.push_back(pool_snapshot{unsigned(index)});
        pool = pools.end();
        std::advance(pool, -1);
        pool->ptr.reset(new pool_snapshot_data<Component>);
    }

    if constexpr(std::is_empty_v<Component>) {
        std::static_pointer_cast<pool_snapshot_data<Component>>(pool->ptr)->pairs.push_back(std::make_pair(entity, Component{}));
    } else {
        auto &comp = registry.get<Component>(entity);
        std::static_pointer_cast<pool_snapshot_data<Component>>(pool->ptr)->pairs.push_back(std::make_pair(entity, comp));
    }
}

template<typename Component>
void insert_entity_component(entt::registry &registry, entt::entity entity, std::vector<pool_snapshot> &pools) {
    auto index = tuple_index_of<Component>(networked_components);
    insert_entity_component<Component>(registry, entity, pools, index);
}

template<typename... Component, typename IndexType, IndexType... Is>
void insert_entity_components(entt::registry &registry, entt::entity entity,
                              std::vector<pool_snapshot> &pools,
                              const std::tuple<Component...> &all_components,
                              std::integer_sequence<IndexType, Is...> int_seq) {
    (insert_entity_component<std::tuple_element_t<Is, std::tuple<Component...>>>(registry, entity, pools, Is), ...);
}

inline void insert_entity_components_default(entt::registry &registry, entt::entity entity,
                                             std::vector<pool_snapshot> &pools) {
    insert_entity_components(registry, entity, pools, networked_components,
                             std::make_index_sequence<std::tuple_size_v<decltype(networked_components)>>{});
}

template<typename SelectComponent, typename... Component>
void insert_select_entity_component(entt::registry &registry, entt::entity entity,
                                     std::vector<pool_snapshot> &pools,
                                     const std::tuple<Component...> &all_components) {
    constexpr auto index = index_of_v<size_t, SelectComponent, Component...>;
    insert_entity_component<std::tuple_element_t<index, std::tuple<Component...>>>(registry, entity, pools, index);
}

template<typename... Component, typename... SelectComponent>
void insert_select_entity_components(entt::registry &registry, entt::entity entity,
                                     std::vector<pool_snapshot> &pools,
                                     const std::tuple<Component...> &all_components,
                                     const std::tuple<SelectComponent...> &select_components) {
    (insert_select_entity_component<SelectComponent>(registry, entity, pools, all_components), ...);
}

inline void insert_transient_components_default(entt::registry &registry, entt::entity entity,
                                                std::vector<pool_snapshot> &pools) {
    insert_select_entity_components(registry, entity, pools, networked_components, transient_components);
}

}

#endif // EDYN_NETWORKING_PACKET_POOL_SNAPSHOT_HPP
