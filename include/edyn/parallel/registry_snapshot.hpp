#ifndef EDYN_PARALLEL_REGISTRY_SNAPSHOT_HPP
#define EDYN_PARALLEL_REGISTRY_SNAPSHOT_HPP

#include <tuple>
#include <vector>
#include <utility>
#include <algorithm>
#include <unordered_set>
#include <unordered_map>
#include <entt/fwd.hpp>
#include "edyn/util/entity_map.hpp"
#include "edyn/util/tuple.hpp"
#include "edyn/comp.hpp"

namespace edyn {

class registry_snapshot_builder;

template<typename Component>
struct updated_components {
    std::vector<std::pair<entt::entity, Component>> value;
};

template<typename Component>
struct destroyed_components {
    std::unordered_set<entt::entity> value;
};

class registry_snapshot {

    template<typename Component>
    void import_child_entity(const entity_map &map, Component &instance) const {
        /* if constexpr(!std::is_same_v<Other, Type>) {
            return;
        } else if constexpr(std::is_same_v<Member, entt::entity>) {
            instance.*member = map.remloc(instance.*member);
        } else {
            // Attempt to use member as a container of entities.
            for(auto &ent : instance.*member) {
                ent = map.remloc(ent);
            }
        } */
    }

    template<typename Component>
    void import_updated(entt::registry &registry, const entity_map &map) const {
        const auto &pairs = std::get<updated_components<Component>>(m_updated_components).value;

        for (auto &pair : pairs) {
            auto remote_entity = pair.first;
            auto local_entity = map.remloc(remote_entity);
            if (!registry.valid(local_entity)) continue;
            auto comp = pair.second;
            import_child_entity(map, comp);
            registry.emplace_or_replace<Component>(local_entity, comp);
        }
    }

    template<typename... Component>
    void import_updated(entt::registry &registry, const entity_map &map, [[maybe_unused]] std::tuple<Component...>) const {
        (import_updated<Component>(registry, map), ...);
    }

    template<typename Component>
    void import_destroyed(entt::registry &registry, const entity_map &map) const {
        using element_type = destroyed_components<Component>;
        const auto &entities = std::get<element_type>(m_destroyed_components).value;

        for (auto remote_entity : entities) {
            auto local_entity = map.locrem(remote_entity);

            if (registry.valid(local_entity) && registry.has<Component>(local_entity)) {
                registry.remove<Component>(local_entity);
            }
        }
    }

    template<typename... Component>
    void import_destroyed(entt::registry &registry, const entity_map &map, [[maybe_unused]] std::tuple<Component...>) const {
        (import_destroyed<Component>(registry, map), ...);
    }

public:
    /**
     * Imports this snapshot into a registry by mapping the entities into the domain
     * of the target registry according to the provided `entity_map`.
     */
    void import(entt::registry &registry, entity_map &map) const;

    friend class registry_snapshot_builder;

    double m_timestamp;

private:
    std::vector<std::pair<entt::entity, entt::entity>> m_remloc_entity_pairs;
    std::unordered_set<entt::entity> m_updated_entities;
    std::unordered_set<entt::entity> m_destroyed_entities;
    map_tuple<updated_components, all_components>::type m_updated_components;
    map_tuple<destroyed_components, all_components>::type m_destroyed_components;
};

class registry_snapshot_builder {
    void insert_entity_mapping(entt::entity entity);

    template<typename Component>
    void insert_entity_mapping(const Component &comp) {
        /* if constexpr(!std::is_same_v<Component, Type>) {
            return;
        } else if constexpr(std::is_same_v<Member, entt::entity>) {
            insert_entity_mapping(comp.*member);
        } else {
            // Attempt to use member as a container of entities.
            for(auto ent : comp.*member) {
                insert_entity_mapping(ent);
            }
        } */

        /* entt::resolve<Component>().data([&] (entt::meta_data &data) {

        }); */
    }

public:
    registry_snapshot_builder(entity_map &map)
        : m_entity_map(&map)
    {}

    /**
     * Adds a component to be updated by the snapshot.
     */
    template<typename... Component>
    void updated(entt::entity entity, const Component &... comp) {
        insert_entity_mapping(entity);

        // Also add entity pairs for child entity members.
        (insert_entity_mapping(comp), ...);

        m_snapshot.m_updated_entities.insert(entity);
        (std::get<updated_components<Component>>(m_snapshot.m_updated_components).value.push_back(std::make_pair(entity, comp)), ...);
    }

    template<typename... Component>
    void updated(entt::entity entity, const entt::registry &registry) {
        if constexpr(sizeof...(Component) <= 1) {
            if constexpr(std::conjunction_v<entt::is_eto_eligible<Component>...>) {
                insert_entity_mapping(entity);
            } else {
                (updated<Component>(entity, registry.get<Component>(entity)), ...);
            }
        } else {
            (updated<Component>(entity, registry), ...);
        }
    }

    template<typename... Component>
    void maybe_updated(entt::entity entity, const entt::registry &registry) {
        ((registry.has<Component>(entity) ? updated<Component>(entity, registry) : (void)0), ...);
    }

    template<typename... Component>
    void maybe_updated(entt::entity entity, const entt::registry &registry, [[maybe_unused]] std::tuple<Component...>) {
        maybe_updated<Component...>(entity, registry);
    }

    /**
     * Marks a component as deleted in this snapshot.
     */
    template<typename... Component>
    void destroyed(entt::entity entity) {
        insert_entity_mapping(entity);

        if constexpr(sizeof...(Component) == 0) {
            m_snapshot.m_destroyed_entities.insert(entity);
        } else {
            (std::get<destroyed_components<Component>>(m_snapshot.m_destroyed_components).value.insert(entity), ...);
        }
    }

    template<typename... Component>
    void destroyed(entt::entity entity, [[maybe_unused]] std::tuple<Component...>) {
        destroyed<Component...>(entity);
    }

    void clear() {
        m_snapshot = {};
    }

    auto & get_snapshot() {
        return m_snapshot;
    }

private:
    entity_map *m_entity_map;
    registry_snapshot m_snapshot;
};

}

#endif // EDYN_PARALLEL_REGISTRY_SNAPSHOT_HPP