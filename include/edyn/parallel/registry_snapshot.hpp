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
    std::unordered_map<entt::entity, Component> value;
};

template<typename Component>
struct destroyed_components {
    std::unordered_set<entt::entity> value;
};

class registry_snapshot {

    void import_created(entt::registry &, entity_map &) const;

    void import_child_entity_sequence(entt::registry &, entity_map &, 
                                      entt::meta_sequence_container *old_seq, 
                                      entt::meta_sequence_container &new_seq) const;

    template<typename Component>
    void import_child_entity(entt::registry &registry, entity_map &map, Component *old_comp, Component &new_comp) const {
        auto range = entt::resolve<Component>().data();
        for (entt::meta_data data : range) {
            if (data.type() == entt::resolve<entt::entity>()) {
                auto new_handle = entt::meta_handle(new_comp);
                auto remote_entity = data.get(new_handle).cast<entt::entity>();

                if (map.has_rem(remote_entity)) {
                    auto local_entity = map.remloc(remote_entity);
                    if (registry.valid(local_entity)) {
                        data.set(new_handle, local_entity);
                    } else {
                        data.set(new_handle, entt::entity{entt::null});
                    }
                }
            } else if (data.type().is_sequence_container()) {
                auto new_handle = entt::meta_handle(new_comp);
                auto new_seq = data.get(new_handle).as_sequence_container();

                if (old_comp) {
                    auto old_handle = entt::meta_handle(*old_comp);
                    auto old_seq = data.get(old_handle).as_sequence_container();
                    import_child_entity_sequence(registry, map, &old_seq, new_seq);
                } else {
                    import_child_entity_sequence(registry, map, nullptr, new_seq);
                }
            }
        }
    }

    template<typename Component>
    void import_updated(entt::registry &registry, entity_map &map) const {
        const auto &pairs = std::get<updated_components<Component>>(m_updated_components).value;

        for (auto &pair : pairs) {
            auto remote_entity = pair.first;
            if (!map.has_rem(remote_entity)) continue;
            auto local_entity = map.remloc(remote_entity);
            if (!registry.valid(local_entity)) continue;
            auto new_component = pair.second;

            if constexpr(std::is_empty_v<Component>) {
                if (!registry.has<Component>(local_entity)) {
                    registry.emplace<Component>(local_entity);
                }
            } else {
                auto *old_component = registry.try_get<Component>(local_entity);
                import_child_entity(registry, map, old_component, new_component);
                registry.emplace_or_replace<Component>(local_entity, new_component);
            }
        }
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
     * Imports this snapshot into a registry by mapping the entities into the domain
     * of the target registry according to the provided `entity_map`.
     */
    void import(entt::registry &, entity_map &) const;

    const auto created() const { return m_created_entities; }

    friend class registry_snapshot_builder;

    double m_timestamp;

    std::vector<std::unordered_set<entt::entity>> m_split_connected_components;
    
private:
    entity_map m_entity_map;
    std::unordered_set<entt::entity> m_created_entities;
    std::unordered_set<entt::entity> m_destroyed_entities;
    map_tuple<updated_components, all_components>::type m_updated_components;
    map_tuple<destroyed_components, all_components>::type m_destroyed_components;
};

class registry_snapshot_builder {
public:
    registry_snapshot_builder(entity_map &map)
        : m_entity_map(&map)
    {}

    void insert_entity_mapping(entt::entity);

    void created(entt::entity entity) {
        m_snapshot.m_created_entities.insert(entity);
    }

    /**
     * Adds a component to be updated by the snapshot.
     */
    template<typename... Component>
    void updated(entt::entity entity, Component &... comp) {
        (std::get<updated_components<Component>>(m_snapshot.m_updated_components).value.insert_or_assign(entity, comp), ...);
    }

    template<typename... Component>
    void updated(entt::entity entity, entt::registry &registry) {
        if constexpr(sizeof...(Component) <= 1) {
            if constexpr(std::conjunction_v<entt::is_eto_eligible<Component>...>) {
                (std::get<updated_components<Component>>(m_snapshot.m_updated_components).value.insert_or_assign(entity, Component{}), ...);
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
     * Marks a component as deleted in this snapshot.
     */
    template<typename... Component>
    void destroyed(entt::entity entity) {
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

    void split(const std::unordered_set<entt::entity> &connected_component) {
        m_snapshot.m_split_connected_components.push_back(connected_component);
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