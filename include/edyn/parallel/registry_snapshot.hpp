#ifndef EDYN_PARALLEL_REGISTRY_SNAPSHOT_HPP
#define EDYN_PARALLEL_REGISTRY_SNAPSHOT_HPP

#include <tuple>
#include <vector>
#include <utility>
#include <unordered_set>
#include <unordered_map>
#include <entt/fwd.hpp>
#include "edyn/util/entity_map.hpp"

namespace edyn {

template<typename... Component>
struct registry_snapshot {

    template<typename Other, typename Type, typename Member>
    void export_child_entity(const entity_map &map, Other &instance, Member Type:: *member) const {
        if constexpr(!std::is_same_v<Other, Type>) {
            return;
        } else if constexpr(std::is_same_v<Member, entt::entity>) {
            instance.*member = map.locrem(instance.*member);
        } else {
            // Attempt to use member as a container of entities.
            for(auto &ent : instance.*member) {
                ent = map.locrem(ent);
            }
        }
    }

    template<typename Other, typename Type, typename Member>
    void import_child_entity(const entity_map &map, Other &instance, Member Type:: *member) const {
        if constexpr(!std::is_same_v<Other, Type>) {
            return;
        } else if constexpr(std::is_same_v<Member, entt::entity>) {
            instance.*member = map.remloc(instance.*member);
        } else {
            // Attempt to use member as a container of entities.
            for(auto &ent : instance.*member) {
                ent = map.remloc(ent);
            }
        }
    }

    template<typename Comp, typename... Type, typename... Member>
    void map_updated(registry_snapshot<Component...> &snapshot, const entity_map &map, Member Type:: *...member) const {
        using element_type = std::vector<std::pair<entt::entity, Comp>>;
        const auto &pairs = std::get<element_type>(m_updated_components);
        auto &export_pairs = std::get<element_type>(snapshot.m_updated_components);

        for (auto &pair : pairs) {
            auto entity = pair.first;
            if (map.has_loc(entity)) {
                auto comp = pair.second;
                (export_child_entity(map, comp, member), ...);
                export_pairs.push_back(std::make_pair(map.locrem(entity), comp));
            }
        }
    }

    template<typename Comp>
    void map_destroyed(registry_snapshot<Component...> &snapshot, const entity_map &map) const {
        using element_type = destroyed_components<Comp>;
        const auto &entities = std::get<element_type>(m_destroyed_components).value;
        auto &export_entities = std::get<element_type>(snapshot.m_destroyed_components).value;

        for (auto entity : entities) {
            if (map.has_loc(entity)) {
                export_entities.insert(map.locrem(entity));
            }
        }
    }

    template<typename Comp, typename... Type, typename... Member>
    void import_updated(entt::registry &registry, const entity_map &map, Member Type:: *...member) const {
        using element_type = std::vector<std::pair<entt::entity, Comp>>;
        const auto &pairs = std::get<element_type>(m_updated_components);

        for (auto &pair : pairs) {
            auto entity = pair.first;
            if (map.has_rem(entity)) {
                auto comp = pair.second;
                (import_child_entity(map, comp, member), ...);
                registry.assign_or_replace<Comp>(map.remloc(entity), comp);
            }
        }
    }

    template<typename Comp>
    void import_destroyed(entt::registry &registry, const entity_map &map) const {
        using element_type = destroyed_components<Comp>;
        const auto &entities = std::get<element_type>(m_destroyed_components).value;

        for (auto remote_entity : entities) {
            if (map.has_rem(remote_entity)) {
                auto local_entity = map.locrem(remote_entity);

                if (registry.valid(local_entity)) {
                    registry.remove<Comp>(local_entity);
                }
            }
        }
    }

    template<typename Comp>
    void load_updated(entt::registry &registry) const {
        using element_type = std::vector<std::pair<entt::entity, Comp>>;
        const auto &pairs = std::get<element_type>(m_updated_components);

        for (auto &pair : pairs) {
            auto entity = pair.first;
            auto &comp = pair.second;
            registry.assign_or_replace<Comp>(entity, comp);
        }
    }

    template<typename Comp>
    void load_destroyed(entt::registry &registry) const {
        using element_type = destroyed_components<Comp>;
        const auto &entities = std::get<element_type>(m_destroyed_components).value;

        for (auto entity : entities) {
            if (registry.valid(entity)) {
                registry.remove<Comp>(entity);
            }
        }
    }

public:
    using component_tuple_t = std::tuple<Component...>;

    registry_snapshot() {}
    registry_snapshot([[maybe_unused]] component_tuple_t) {}
    registry_snapshot(const registry_snapshot<Component...> &) = default;

    /**
     * Adds a component to be updated by the snapshot.
     */
    template<typename... Comp>
    void updated(entt::entity entity, const Comp&... comp) {
        m_updated_entities.insert(entity);
        (std::get<std::vector<std::pair<entt::entity, Comp>>>(m_updated_components).push_back(std::make_pair(entity, comp)), ...);
    }

    /**
     * Marks a component as deleted in this snapshot.
     */
    template<typename... Comp>
    void destroyed(entt::entity entity) {
        (std::get<destroyed_components<Comp>>(m_destroyed_components).value.insert(entity), ...);
    }

    /**
     * Maps entities from this snapshot into another domain.
     */
    template<typename... Type, typename... Member>
    auto map_entities(const entity_map &map, Member Type:: *...member) const {
        auto snapshot = registry_snapshot<Component...>{};

        for (auto entity : m_updated_entities) {
            if (map.has_loc(entity)) {
                snapshot.m_updated_entities.insert(map.locrem(entity));
            }
        }

        (map_updated<Component>(snapshot, map, member...), ...);
        (map_destroyed<Component>(snapshot, map), ...);

        return snapshot;
    }

    /**
     * Imports this snapshot into a registry by mapping the entities into the domain
     * of the target registry according to the provided `entity_map`.
     */
    template<typename... Type, typename... Member>
    void import(entt::registry &registry, entity_map &map, Member Type:: *...member) const {
        for (auto remote_entity : m_updated_entities) {
            if (!map.has_rem(remote_entity)) {
                auto entity = registry.create();
                map.insert(remote_entity, entity);
            }
        }

        (import_updated<Component>(registry, map, member...), ...);
        (import_destroyed<Component>(registry, map), ...);
    }

    /**
     * Loads this snapshot into the target registry.
     */
    void load(entt::registry &registry) const {
        (load_updated<Component>(registry), ...);
        (load_destroyed<Component>(registry), ...);
    }

private:
    std::unordered_set<entt::entity> m_updated_entities;
    std::tuple<std::vector<std::pair<entt::entity, Component>>...> m_updated_components;

    template<typename Comp>
    struct destroyed_components {
        std::unordered_set<entt::entity> value;
    };
    std::tuple<destroyed_components<Component>...> m_destroyed_components;
};

}

#endif // EDYN_PARALLEL_REGISTRY_SNAPSHOT_HPP