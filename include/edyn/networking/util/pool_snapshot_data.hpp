#ifndef EDYN_NETWORKING_UTIL_POOL_SNAPSHOT_DATA_HPP
#define EDYN_NETWORKING_UTIL_POOL_SNAPSHOT_DATA_HPP

#include <iterator>
#include <memory>
#include <vector>
#include <utility>
#include <entt/entity/fwd.hpp>
#include "edyn/comp/merge_component.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/replication/map_child_entity.hpp"
#include "edyn/serialization/std_s11n.hpp"
#include "edyn/serialization/memory_archive.hpp"
#include "edyn/replication/entity_map.hpp"
#include "edyn/config/config.h"

namespace edyn {

struct pool_snapshot_data {
    // Snapshot is not expected to have more than 256 entities.
    using index_type = uint8_t;
    std::vector<index_type> entity_indices;

    virtual ~pool_snapshot_data() = default;
    virtual void convert_remloc(const entt::registry &registry, const entity_map &emap) = 0;
    virtual void write(memory_output_archive &archive) = 0;
    virtual void read(memory_input_archive &archive) = 0;

    virtual void replace_into_registry(entt::registry &registry,
                                       const std::vector<entt::entity> &entities,
                                       const entity_map &emap) = 0;

    virtual void replace_into_registry(entt::registry &registry,
                                       const std::vector<entt::entity> &entities) = 0;

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
        // TODO: should be no-op if component type has no child entities.
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
                        registry.patch<Component>(local_entity, [&](auto &&current) {
                            merge_component(current, comp);
                        });
                    }
                }
            }
        }
    }

    void replace_into_registry(entt::registry &registry,
                               const std::vector<entt::entity> &pool_entities) override {
        if constexpr(!is_empty_type) {
            EDYN_ASSERT(entity_indices.size() == components.size());

            for (size_t i = 0; i < entity_indices.size(); ++i) {
                auto entity_index = entity_indices[i];
                auto entity = pool_entities[entity_index];

                if (registry.valid(entity) && registry.all_of<Component>(entity)) {
                    auto &comp = components[i];
                    registry.patch<Component>(entity, [&](auto &&current) {
                        merge_component(current, comp);
                    });
                }
            }
        }
    }

    void insert_single(const entt::registry &registry, entt::entity entity,
                       std::vector<entt::entity> &pool_entities) {
        EDYN_ASSERT((registry.all_of<networked_tag, Component>(entity)));

        auto found_it = std::find(pool_entities.begin(), pool_entities.end(), entity);
        index_type idx;

        if (found_it != pool_entities.end()) {
            idx = std::distance(pool_entities.begin(), found_it);
        } else {
            idx = pool_entities.size();
            pool_entities.push_back(entity);
        }

        entity_indices.push_back(idx);

        if constexpr(!is_empty_type) {
            auto &comp = registry.get<Component>(entity);
            components.push_back(comp);
        }
    }

    template<typename It>
    void insert(const entt::registry &registry, It first, It last,
                std::vector<entt::entity> &pool_entities) {
        auto view = registry.view<Component>();

        for (; first != last; ++first) {
            auto entity = *first;
            EDYN_ASSERT((registry.all_of<networked_tag, Component>(entity)));

            auto found_it = std::find(pool_entities.begin(), pool_entities.end(), entity);
            index_type idx;

            if (found_it != pool_entities.end()) {
                idx = std::distance(pool_entities.begin(), found_it);
            } else {
                idx = pool_entities.size();
                pool_entities.push_back(entity);
            }

            entity_indices.push_back(idx);

            if constexpr(!is_empty_type) {
                auto [comp] = view.get(entity);
                components.push_back(comp);
            }
        }
    }

    entt::id_type get_type_id() const override {
        return entt::type_index<Component>::value();
    }
};

}

#endif // EDYN_NETWORKING_UTIL_POOL_SNAPSHOT_DATA_HPP
