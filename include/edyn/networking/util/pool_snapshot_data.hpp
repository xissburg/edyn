#ifndef EDYN_NETWORKING_UTIL_POOL_SNAPSHOT_DATA_HPP
#define EDYN_NETWORKING_UTIL_POOL_SNAPSHOT_DATA_HPP

#include <iterator>
#include <memory>
#include <vector>
#include <utility>
#include <entt/entity/fwd.hpp>
#include "edyn/parallel/map_child_entity.hpp"
#include "edyn/serialization/memory_archive.hpp"
#include "edyn/util/entity_map.hpp"
#include "edyn/config/config.h"
#include "edyn/comp/tag.hpp"

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

                if (emap.contains(remote_entity)) {
                    auto local_entity = emap.at(remote_entity);

                    if (registry.valid(local_entity) && registry.all_of<Component>(local_entity)) {
                        auto &comp = components[i];
                        internal::map_child_entity(registry, emap, comp);
                        registry.replace<Component>(local_entity, comp);
                    }
                }
            }
        }
    }

    void insert_all(const entt::registry &registry,
                    const std::vector<entt::entity> &pool_entities) {
        auto view = registry.view<Component, networked_tag>();

        for (index_type idx = 0; idx < pool_entities.size(); ++idx) {
            auto entity = pool_entities[idx];

            if (view.contains(entity)) {
                entity_indices.push_back(idx);

                if constexpr(!is_empty_type) {
                    auto [comp] = view.get(entity);
                    components.push_back(comp);
                }
            }
        }
    }

    void insert_single(const entt::registry &registry, entt::entity entity,
                       const std::vector<entt::entity> &pool_entities) {
        auto found_it = std::find(pool_entities.begin(), pool_entities.end(), entity);
        auto view = registry.view<Component>();

        if (found_it == pool_entities.end() || !view.contains(entity)) {
            return;
        }

        EDYN_ASSERT(registry.all_of<networked_tag>(entity));

        auto idx = std::distance(pool_entities.begin(), found_it);
        entity_indices.push_back(idx);

        if constexpr(!is_empty_type) {
            auto [comp] = view.get(entity);
            components.push_back(comp);
        }
    }

    template<typename It>
    void insert(const entt::registry &registry, It first, It last,
                const std::vector<entt::entity> &pool_entities) {
        auto view = registry.view<Component>();

        for (; first != last; ++first) {
            auto entity = *first;
            auto found_it = std::find(pool_entities.begin(), pool_entities.end(), entity);

            if (found_it == pool_entities.end() || !view.contains(entity)) {
                continue;
            }

            EDYN_ASSERT(registry.all_of<networked_tag>(entity));

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

}

#endif // EDYN_NETWORKING_UTIL_POOL_SNAPSHOT_DATA_HPP
