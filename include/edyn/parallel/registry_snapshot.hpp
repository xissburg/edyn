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
class registry_snapshot_builder;

template<typename Comp>
struct destroyed_components {
    std::unordered_set<entt::entity> value;
};

template<typename... Component>
class registry_snapshot {

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
    void import_updated(entt::registry &registry, const entity_map &map, Member Type:: *...member) const {
        using element_type = std::vector<std::pair<entt::entity, Comp>>;
        const auto &pairs = std::get<element_type>(m_updated_components);

        for (auto &pair : pairs) {
            auto remote_entity = pair.first;
            auto local_entity = map.remloc(remote_entity);
            if (!registry.valid(local_entity)) continue;
            auto comp = pair.second;
            (import_child_entity(map, comp, member), ...);
            registry.assign_or_replace<Comp>(local_entity, comp);
        }
    }

    template<typename Comp>
    void import_destroyed(entt::registry &registry, const entity_map &map) const {
        using element_type = destroyed_components<Comp>;
        const auto &entities = std::get<element_type>(m_destroyed_components).value;

        for (auto remote_entity : entities) {
            auto local_entity = map.locrem(remote_entity);

            if (registry.valid(local_entity) && registry.has<Comp>(local_entity)) {
                registry.remove<Comp>(local_entity);
            }
        }
    }

public:
    using component_tuple_t = std::tuple<Component...>;

    registry_snapshot() {}
    registry_snapshot([[maybe_unused]] component_tuple_t) {}
    registry_snapshot(const registry_snapshot<Component...> &) = default;

    /**
     * Imports this snapshot into a registry by mapping the entities into the domain
     * of the target registry according to the provided `entity_map`.
     */
    template<typename... Type, typename... Member>
    void import(entt::registry &registry, entity_map &map, Member Type:: *...member) const {
        for (auto &pair : m_remloc_entity_pairs) {
            auto remote_entity = pair.first;
            auto local_entity = pair.second;

            if (!map.has_rem(remote_entity)) {
                if (local_entity != entt::null) {
                    map.insert(remote_entity, local_entity);
                } else {
                    map.insert(remote_entity, registry.create());
                }
            }
        }

        (import_updated<Component>(registry, map, member...), ...);
        (import_destroyed<Component>(registry, map), ...);

        for (auto entity : m_destroyed_entities) {
            if (!map.has_rem(entity)) continue;
            auto local_entity = map.remloc(entity);
            map.erase_rem(entity);

            if (registry.valid(local_entity)) {
                registry.destroy(local_entity);
            }
        }
    }

    template<typename... Type, typename... Member>
    void import(entt::registry &registry, entity_map &map, std::tuple<Member Type:: *...> members) const {
        std::apply([&] (auto &&... member) {
            import(registry, map, member...);
        }, members);
    }

    friend class registry_snapshot_builder<Component...>;

private:
    std::vector<std::pair<entt::entity, entt::entity>> m_remloc_entity_pairs;
    std::unordered_set<entt::entity> m_updated_entities;
    std::unordered_set<entt::entity> m_destroyed_entities;
    std::tuple<std::vector<std::pair<entt::entity, Component>>...> m_updated_components;
    std::tuple<destroyed_components<Component>...> m_destroyed_components;
};

template<typename... Component>
class registry_snapshot_builder {
    void insert_entity_mapping(entt::entity entity) {
        auto found_it = std::find_if(
            m_snapshot.m_remloc_entity_pairs.begin(), 
            m_snapshot.m_remloc_entity_pairs.end(), 
            [entity] (auto &pair) { return entity == pair.first; });

        if (found_it != m_snapshot.m_remloc_entity_pairs.end()) {
            return;
        }
        
        if (m_entity_map->has_loc(entity)) {
            auto remote_entity = m_entity_map->locrem(entity);
            m_snapshot.m_remloc_entity_pairs.emplace_back(entity, remote_entity);
        } else {
            m_snapshot.m_remloc_entity_pairs.emplace_back(entity, entt::null);
        }
    }

    template<typename Comp, typename Type, typename Member>
    void insert_entity_mapping(const Comp &comp, Member Type:: *member) {
        if constexpr(!std::is_same_v<Comp, Type>) {
            return;
        } else if constexpr(std::is_same_v<Member, entt::entity>) {
            insert_entity_mapping(comp.*member);
        } else {
            // Attempt to use member as a container of entities.
            for(auto ent : comp.*member) {
                insert_entity_mapping(ent);
            }
        }
    }

    template<typename Comp, typename... Type, typename... Member>
    void insert_entity_mapping(const Comp &comp, Member Type:: *...member) {
        (insert_entity_mapping(comp, member), ...);
    }

public:
    using component_tuple_t = std::tuple<Component...>;
    using registry_snapshot_t = registry_snapshot<Component...>;

    registry_snapshot_builder(entity_map &map)
        : m_entity_map(&map)
    {}

    registry_snapshot_builder(entity_map &map, [[maybe_unused]] component_tuple_t)
        : m_entity_map(&map)
    {}

    registry_snapshot_builder(const registry_snapshot_builder<Component...> &) = default;

    /**
     * Adds a component to be updated by the snapshot.
     */
    template<typename... Comp, typename... Type, typename... Member>
    void updated(entt::entity entity, const Comp &... comp, Member Type:: *...member) {
        insert_entity_mapping(entity);

        // Also add entity pairs for members.
        (insert_entity_mapping(comp, member...), ...);

        m_snapshot.m_updated_entities.insert(entity);
        (std::get<std::vector<std::pair<entt::entity, Comp>>>(m_snapshot.m_updated_components).push_back(std::make_pair(entity, comp)), ...);
    }
    
    template<typename... Comp, typename... Type, typename... Member>
    void updated(entt::entity entity, const Comp &... comp, std::tuple<Member Type:: *...> members) {
        std::apply([&] (auto &&... member) {
            updated<Comp...>(entity, comp..., member...);
        }, members);
    }

    template<typename... Comp, typename... Type, typename... Member>
    void maybe_updated(entt::entity entity, const entt::registry &registry, Member Type:: *...member) {
        ((registry.has<Comp>(entity) ? updated<Comp>(entity, registry.get<Comp>(entity), member...) : (void)0), ...);
    }

    template<typename... Comp, typename... Type, typename... Member>
    void maybe_updated(entt::entity entity, const entt::registry &registry, [[maybe_unused]] std::tuple<Comp...>, Member Type:: *...member) {
        maybe_updated<Comp...>(entity, registry, member...);
    }

    template<typename... Comp, typename... Type, typename... Member>
    void maybe_updated(entt::entity entity, const entt::registry &registry, std::tuple<Member Type:: *...> members) {
        std::apply([&] (auto &&... member) {
            maybe_updated<Comp...>(entity, registry, member...);
        }, members);
    }

    template<typename... Comp, typename... Type, typename... Member>
    void maybe_updated(entt::entity entity, const entt::registry &registry, std::tuple<Comp...> comp_tuple, std::tuple<Member Type:: *...> members) {
        std::apply([&] (auto &&... member) {
            maybe_updated<Comp...>(entity, registry, comp_tuple, member...);
        }, members);
    }

    /**
     * Marks a component as deleted in this snapshot.
     */
    template<typename... Comp>
    void destroyed(entt::entity entity) {
        insert_entity_mapping(entity);

        if constexpr(sizeof...(Comp) == 0) {
            m_snapshot.m_destroyed_entities.insert(entity);
        } else {
            (std::get<destroyed_components<Comp>>(m_snapshot.m_destroyed_components).value.insert(entity), ...);
        }
    }

    template<typename... Comp>
    void destroyed(entt::entity entity, [[maybe_unused]] std::tuple<Comp...>) {
        destroyed<Comp...>(entity);
    }

    void clear() {
        m_snapshot = {};
    }

    auto & get_snapshot() {
        return m_snapshot;
    }

private:
    entity_map *m_entity_map;
    registry_snapshot<Component...> m_snapshot;
};

}

#endif // EDYN_PARALLEL_REGISTRY_SNAPSHOT_HPP