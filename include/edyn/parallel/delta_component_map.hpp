#ifndef EDYN_PARALLEL_DELTA_COMPONENT_MAP_HPP
#define EDYN_PARALLEL_DELTA_COMPONENT_MAP_HPP

#include <utility>
#include <unordered_map>
#include <entt/fwd.hpp>
#include "edyn/util/entity_set.hpp"
#include "edyn/parallel/merge/merge_component.hpp"
#include "edyn/parallel/merge/merge_island.hpp"
#include "edyn/parallel/merge/merge_contact_point.hpp"
#include "edyn/parallel/merge/merge_contact_manifold.hpp"
#include "edyn/parallel/merge/merge_constraint_row.hpp"
#include "edyn/parallel/merge/merge_constraint.hpp"
#include "edyn/parallel/merge/merge_gravity.hpp"
#include "edyn/parallel/merge/merge_tree_view.hpp"

namespace edyn {

class registry_delta;

struct component_map_base {
    virtual ~component_map_base() {}
    virtual void import(const registry_delta &, entt::registry &, entity_map &) = 0;
    virtual bool empty() const = 0;
    virtual void clear() = 0;
};

template<typename Component>
struct updated_component_map: public component_map_base {
    std::unordered_map<entt::entity, Component> pairs;

    void insert(entt::entity entity, const Component &comp) {
        pairs.insert_or_assign(entity, comp);
    }

    void import(const registry_delta &delta, entt::registry &registry, entity_map &map) override {
        auto ctx = merge_context{&registry, &map, &delta};
        auto view = registry.view<Component>();

        for (auto &pair : pairs) {
            auto remote_entity = pair.first;
            if (!map.has_rem(remote_entity)) continue;
            auto local_entity = map.remloc(remote_entity);
            if (!registry.valid(local_entity)) continue;

            if constexpr(!std::is_empty_v<Component>) {
                auto &old_component = view.get(local_entity);
                merge<merge_type::updated>(&old_component, pair.second, ctx);
                registry.replace<Component>(local_entity, pair.second);
            }
        }
    }

    bool empty() const override {
        return pairs.empty();
    }

    void clear() override {
        pairs.clear();
    }
};

template<typename Component>
struct created_component_map: public component_map_base {
    std::unordered_map<entt::entity, Component> pairs;

    void insert(entt::entity entity, const Component &comp) {
        pairs.insert_or_assign(entity, comp);
    }

    void import(const registry_delta &delta, entt::registry &registry, entity_map &map) override {
        auto ctx = merge_context{&registry, &map, &delta};

        for (auto &pair : pairs) {
            auto remote_entity = pair.first;
            if (!map.has_rem(remote_entity)) continue;
            auto local_entity = map.remloc(remote_entity);
            if (!registry.valid(local_entity)) continue;
            if (registry.has<Component>(local_entity)) continue;

            if constexpr(std::is_empty_v<Component>) {
                registry.emplace<Component>(local_entity);
            } else {
                merge<merge_type::created>(static_cast<Component *>(nullptr), pair.second, ctx);
                registry.emplace<Component>(local_entity, pair.second);
            }
        }
    }

    bool empty() const override {
        return pairs.empty();
    }

    void clear() override {
        pairs.clear();
    }
};

template<typename Component>
struct destroyed_component_map: public component_map_base {
    entity_set entities;

    void insert(entt::entity entity) {
        entities.insert(entity);
    }

    void import(const registry_delta &, entt::registry &registry, entity_map &map) override {
        for (auto remote_entity : entities) {
            if (!map.has_rem(remote_entity)) continue;
            auto local_entity = map.remloc(remote_entity);
            if (!registry.valid(local_entity)) continue;

            if (registry.has<Component>(local_entity)) {
                registry.remove<Component>(local_entity);
            }
        }
    }

    bool empty() const override {
        return entities.empty();
    }

    void clear() override {
        entities.clear();
    }
};

}

#endif // EDYN_PARALLEL_DELTA_COMPONENT_MAP_HPP
