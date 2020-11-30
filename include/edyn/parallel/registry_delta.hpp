#ifndef EDYN_PARALLEL_REGISTRY_DELTA_HPP
#define EDYN_PARALLEL_REGISTRY_DELTA_HPP

#include <tuple>
#include <vector>
#include <utility>
#include <algorithm>
#include <unordered_set>
#include <unordered_map>
#include <entt/fwd.hpp>
#include "edyn/comp.hpp"
#include "edyn/util/tuple.hpp"
#include "edyn/util/entity_map.hpp"
#include "edyn/parallel/merge/merge_component.hpp"
#include "edyn/parallel/merge/merge_island.hpp"
#include "edyn/parallel/merge/merge_contact_point.hpp"
#include "edyn/parallel/merge/merge_contact_manifold.hpp"
#include "edyn/parallel/merge/merge_constraint_row.hpp"
#include "edyn/parallel/merge/merge_constraint.hpp"
#include "edyn/parallel/merge/merge_gravity.hpp"

namespace edyn {

class registry_delta_builder;

template<typename Component>
struct component_map {
    std::unordered_map<entt::entity, Component> value;
};

template<typename Component>
struct destroyed_components {
    std::unordered_set<entt::entity> value;
};

class registry_delta {

    void import_created_entities(entt::registry &, entity_map &) const;

    template<typename Component>
    void import_updated(entt::registry &registry, entity_map &map) const {
        const auto &pairs = std::get<component_map<Component>>(m_updated_components).value;
        auto ctx = merge_context{&registry, &map, this};

        for (auto &pair : pairs) {
            auto remote_entity = pair.first;
            if (!map.has_rem(remote_entity)) continue;
            auto local_entity = map.remloc(remote_entity);
            if (!registry.valid(local_entity)) continue;

            if constexpr(!std::is_empty_v<Component>) {
                auto new_component = pair.second;
                auto &old_component = registry.get<Component>(local_entity);
                merge<merge_type::updated>(&old_component, new_component, ctx);
                registry.replace<Component>(local_entity, new_component);
            }
        }
    }
    
    template<typename Component>
    void import_created_components(entt::registry &registry, entity_map &map) const {
        const auto &pairs = std::get<component_map<Component>>(m_created_components).value;
        auto ctx = merge_context{&registry, &map, this};

        for (auto &pair : pairs) {
            auto remote_entity = pair.first;
            if (!map.has_rem(remote_entity)) continue;
            auto local_entity = map.remloc(remote_entity);
            if (!registry.valid(local_entity)) continue;

            if constexpr(std::is_empty_v<Component>) {
                registry.emplace<Component>(local_entity);
            } else {
                auto new_component = pair.second;
                merge<merge_type::created>(static_cast<Component*>(nullptr), new_component, ctx);
                registry.emplace<Component>(local_entity, new_component);
            }
        }
    }

    template<typename... Component>
    void import_created_components(entt::registry &registry, entity_map &map, [[maybe_unused]] std::tuple<Component...>) const {
        (import_created_components<Component>(registry, map), ...);
    }

    template<typename... Component>
    void import_updated(entt::registry &registry, entity_map &map, [[maybe_unused]] std::tuple<Component...>) const {
        (import_updated<Component>(registry, map), ...);
    }

    template<typename Component>
    void import_destroyed(entt::registry &registry, entity_map &map) const {
        using element_type = destroyed_components<Component>;
        const auto &entities = std::get<element_type>(m_destroyed_components).value;

        for (auto remote_entity : entities) {
            if (!map.has_rem(remote_entity)) continue;
            auto local_entity = map.locrem(remote_entity);

            if (registry.valid(local_entity) && registry.has<Component>(local_entity)) {
                registry.remove<Component>(local_entity);
            }
        }
    }

    template<typename... Component>
    void import_destroyed(entt::registry &registry, entity_map &map, [[maybe_unused]] std::tuple<Component...>) const {
        (import_destroyed<Component>(registry, map), ...);
    }

public:
    /**
     * Imports this delta into a registry by mapping the entities into the domain
     * of the target registry according to the provided `entity_map`.
     */
    void import(entt::registry &, entity_map &) const;

    const auto created() const { return m_created_entities; }
    const auto destroyed() const { return m_destroyed_entities; }

    friend class registry_delta_builder;

    double m_timestamp;

    std::vector<std::unordered_set<entt::entity>> m_split_connected_components;
    
private:
    entity_map m_entity_map;
    std::unordered_set<entt::entity> m_created_entities;
    std::unordered_set<entt::entity> m_destroyed_entities;
    map_tuple<component_map, all_components>::type m_created_components;
    map_tuple<component_map, all_components>::type m_updated_components;
    map_tuple<destroyed_components, all_components>::type m_destroyed_components;
};

class registry_delta_builder {
public:
    registry_delta_builder(entity_map &map)
        : m_entity_map(&map)
    {}

    void insert_entity_mapping(entt::entity);

    void created(entt::entity entity) {
        m_delta.m_created_entities.insert(entity);
    }

    template<typename Component, typename... Components>
    void created(entt::entity entity, const Component &comp, const Components &... comps) {
        std::get<component_map<Component>>(m_delta.m_created_components).value.insert_or_assign(entity, comp);
        (std::get<component_map<Components>>(m_delta.m_created_components).value.insert_or_assign(entity, comps), ...);
    }

    template<typename Component>
    void created(entt::entity entity, entt::registry &registry) {
        if constexpr(entt::is_eto_eligible_v<Component>) {
            std::get<component_map<Component>>(m_delta.m_created_components).value.insert_or_assign(entity, Component{});
        } else {
            created<Component>(entity, registry.get<Component>(entity));
        }
    }

    template<typename... Component>
    void created(entt::entity entity, entt::registry &registry, entt::id_type id) {
        ((entt::type_index<Component>::value() == id ? created<Component>(entity, registry) : (void)0), ...);
    }

    template<typename... Component>
    void created(entt::entity entity, entt::registry &registry, entt::id_type id, std::tuple<Component...>) {
        created<Component...>(entity, registry, id);
    }

    template<typename... Component, typename It>
    void created(entt::entity entity, entt::registry &registry, It first, It last, std::tuple<Component...>) {
        for (auto it = first; it != last; ++it) {
            created<Component...>(entity, registry, *it);
        }
    }

    template<typename... Component>
    void maybe_created(entt::entity entity, entt::registry &registry) {
        ((registry.has<Component>(entity) ? created<Component>(entity, registry) : (void)0), ...);
    }

    template<typename... Component>
    void maybe_created(entt::entity entity, entt::registry &registry, [[maybe_unused]] std::tuple<Component...>) {
        maybe_created<Component...>(entity, registry);
    }

    /**
     * Adds a component to be updated by the delta.
     */
    template<typename... Component>
    void updated(entt::entity entity, Component &... comp) {
        (std::get<component_map<Component>>(m_delta.m_updated_components).value.insert_or_assign(entity, comp), ...);
    }

    template<typename... Component>
    void updated(entt::entity entity, entt::registry &registry) {
        if constexpr(sizeof...(Component) <= 1) {
            if constexpr(std::conjunction_v<entt::is_eto_eligible<Component>...>) {
                (std::get<component_map<Component>>(m_delta.m_updated_components).value.insert_or_assign(entity, Component{}), ...);
            } else {
                (updated<Component>(entity, registry.get<Component>(entity)), ...);
            }
        } else {
            (updated<Component>(entity, registry), ...);
        }
    }

    template<typename... Component>
    void updated(entt::entity entity, entt::registry &registry, entt::id_type id) {
        ((entt::type_index<Component>::value() == id ? updated<Component>(entity, registry) : (void)0), ...);
    }

    template<typename... Component>
    void updated(entt::entity entity, entt::registry &registry, entt::id_type id, std::tuple<Component...>) {
        updated<Component...>(entity, registry, id);
    }

    template<typename... Component, typename It>
    void updated(entt::entity entity, entt::registry &registry, It first, It last, std::tuple<Component...>) {
        for (auto it = first; it != last; ++it) {
            updated<Component...>(entity, registry, *it);
        }
    }

    template<typename... Component>
    void maybe_updated(entt::entity entity, entt::registry &registry) {
        ((registry.has<Component>(entity) ? updated<Component>(entity, registry) : (void)0), ...);
    }

    template<typename... Component>
    void maybe_updated(entt::entity entity, entt::registry &registry, [[maybe_unused]] std::tuple<Component...>) {
        maybe_updated<Component...>(entity, registry);
    }

    /**
     * Marks a component as deleted in this delta.
     */
    template<typename... Component>
    void destroyed(entt::entity entity) {
        if constexpr(sizeof...(Component) == 0) {
            m_delta.m_destroyed_entities.insert(entity);
        } else {
            (std::get<destroyed_components<Component>>(m_delta.m_destroyed_components).value.insert(entity), ...);
        }
    }

    template<typename... Component>
    void destroyed(entt::entity entity, [[maybe_unused]] std::tuple<Component...>) {
        destroyed<Component...>(entity);
    }

    void split(const std::unordered_set<entt::entity> &connected_component) {
        m_delta.m_split_connected_components.push_back(connected_component);
    }

    void clear() {
        m_delta = {};
    }

    auto & get_delta() {
        return m_delta;
    }

private:
    entity_map *m_entity_map;
    registry_delta m_delta;
};

}

#endif // EDYN_PARALLEL_REGISTRY_DELTA_HPP