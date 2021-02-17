#ifndef EDYN_PARALLEL_ENTITY_COMPONENT_CONTAINER_HPP
#define EDYN_PARALLEL_ENTITY_COMPONENT_CONTAINER_HPP

#include <vector>
#include <utility>
#include <entt/fwd.hpp>
#include "edyn/parallel/entity_component_map.hpp"
#include "edyn/parallel/merge/merge_component.hpp"
#include "edyn/parallel/merge/merge_island.hpp"
#include "edyn/parallel/merge/merge_contact_point.hpp"
#include "edyn/parallel/merge/merge_contact_manifold.hpp"
#include "edyn/parallel/merge/merge_constraint_row.hpp"
#include "edyn/parallel/merge/merge_constraint.hpp"
#include "edyn/parallel/merge/merge_gravity.hpp"
#include "edyn/parallel/merge/merge_tree_view.hpp"

namespace edyn {

class island_delta;

struct entity_component_container_base {
    virtual ~entity_component_container_base() {}
    virtual void import(const island_delta &, entt::registry &, entity_map &) = 0;
    virtual void load(const entity_component_map_base &) = 0;
    virtual bool empty() const = 0;
    virtual void clear() = 0;
};

template<typename Component>
struct updated_entity_component_container: public entity_component_container_base {
    std::vector<std::pair<entt::entity, Component>> pairs;

    void load(const entity_component_map_base &comp_map_base) override {
        const auto &comp_map = static_cast<const entity_component_map<Component> &>(comp_map_base);
        for (auto &pair : comp_map.map) {
            pairs.push_back(pair);
        }
    }

    void import(const island_delta &delta, entt::registry &registry, entity_map &map) override {
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
struct created_entity_component_container: public entity_component_container_base {
    std::vector<std::pair<entt::entity, Component>> pairs;

    void load(const entity_component_map_base &comp_map_base) override {
        const auto &comp_map = static_cast<const entity_component_map<Component> &>(comp_map_base);
        for (const auto &pair : comp_map.map) {
            pairs.push_back(pair);
        }
    }

    void import(const island_delta &delta, entt::registry &registry, entity_map &map) override {
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
struct destroyed_entity_component_container: public entity_component_container_base {
    std::vector<entt::entity> entities;

    void load(const entity_component_map_base &comp_map_base) override {
        const auto &comp_set = static_cast<const entity_component_set<Component> &>(comp_map_base);
        for (auto entity : comp_set.set) {
            entities.push_back(entity);
        }
    }

    void import(const island_delta &, entt::registry &registry, entity_map &map) override {
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

#endif // EDYN_PARALLEL_ENTITY_COMPONENT_CONTAINER_HPP
