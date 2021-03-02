#ifndef EDYN_PARALLEL_ENTITY_COMPONENT_CONTAINER_HPP
#define EDYN_PARALLEL_ENTITY_COMPONENT_CONTAINER_HPP

#include <vector>
#include <utility>
#include <entt/fwd.hpp>
#include "edyn/parallel/merge/merge_component.hpp"
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
    virtual void reserve(size_t size) = 0;
    virtual bool empty() const = 0;
    virtual void clear() = 0;
};

template<typename Component>
struct updated_entity_component_container: public entity_component_container_base {
    std::vector<std::pair<entt::entity, Component>> pairs;

    void insert(entt::entity entity, const Component &comp) {
        pairs.emplace_back(entity, comp);
    }

    void import(const island_delta &delta, entt::registry &registry, entity_map &map) override {
        auto ctx = merge_context{&registry, &map, &delta};
        auto view = registry.view<Component>();

        for (auto &pair : pairs) {
            auto remote_entity = pair.first;
            auto local_entity = map.remloc(remote_entity);

            if constexpr(!std::is_empty_v<Component>) {
                auto &old_component = view.get(local_entity);
                merge<merge_type::updated>(&old_component, pair.second, ctx);
                registry.replace<Component>(local_entity, pair.second);
            }
        }
    }

    void reserve(size_t size) override {
        pairs.reserve(size);
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

    void insert(entt::entity entity, const Component &comp) {
        pairs.emplace_back(entity, comp);
    }

    void import(const island_delta &delta, entt::registry &registry, entity_map &map) override {
        auto ctx = merge_context{&registry, &map, &delta};
        size_t index = 0;

        while (index < pairs.size()) {
            auto &pair = pairs[index];
            auto remote_entity = pair.first;
            auto local_entity = map.remloc(remote_entity);

            // If it's a duplicate, remove it from the array by swapping with last
            // and popping last. This ensures no duplicates after processing, which
            // can be useful during post processing after import.
            if (registry.has<Component>(local_entity)) {
                pairs[index] = pairs.back();
                pairs.pop_back();
                continue;
            }

            if constexpr(std::is_empty_v<Component>) {
                registry.emplace<Component>(local_entity);
            } else {
                merge<merge_type::created>(static_cast<Component *>(nullptr), pair.second, ctx);
                registry.emplace<Component>(local_entity, pair.second);
            }

            ++index;
        }
    }

    void reserve(size_t size) override {
        pairs.reserve(size);
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

    void import(const island_delta &, entt::registry &registry, entity_map &map) override {
        for (auto remote_entity : entities) {
            if (!map.has_rem(remote_entity)) continue;
            auto local_entity = map.remloc(remote_entity);
            registry.remove<Component>(local_entity);
        }
    }

    void reserve(size_t size) override {
        entities.reserve(size);
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
