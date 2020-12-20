#ifndef EDYN_PARALLEL_REGISTRY_DELTA_HPP
#define EDYN_PARALLEL_REGISTRY_DELTA_HPP

#include <tuple>
#include <vector>
#include <utility>
#include <entt/fwd.hpp>
#include <unordered_map>
#include <entt/core/type_info.hpp>
#include "edyn/util/entity_map.hpp"
#include "edyn/parallel/merge/merge_component.hpp"
#include "edyn/parallel/merge/merge_island.hpp"
#include "edyn/parallel/merge/merge_contact_point.hpp"
#include "edyn/parallel/merge/merge_contact_manifold.hpp"
#include "edyn/parallel/merge/merge_constraint_row.hpp"
#include "edyn/parallel/merge/merge_constraint.hpp"
#include "edyn/parallel/merge/merge_gravity.hpp"

namespace edyn {

class registry_delta;
class registry_delta_builder;

struct component_map_base {
    virtual ~component_map_base() {}
    virtual void import(const registry_delta &, entt::registry &, entity_map &) const = 0;
};

template<typename Component>
struct updated_component_map: public component_map_base {
    std::unordered_map<entt::entity, Component> pairs;

    void insert(entt::entity entity, const Component &comp) {
        pairs.insert_or_assign(entity, comp);
    }

    void import(const registry_delta &delta, entt::registry &registry, entity_map &map) const override {
        auto ctx = merge_context{&registry, &map, &delta};

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
};

template<typename Component>
struct created_component_map: public component_map_base {
    std::unordered_map<entt::entity, Component> pairs;

    void insert(entt::entity entity, const Component &comp) {
        pairs.insert_or_assign(entity, comp);
    }

    void import(const registry_delta &delta, entt::registry &registry, entity_map &map) const override {
        auto ctx = merge_context{&registry, &map, &delta};

        for (auto &pair : pairs) {
            auto remote_entity = pair.first;
            if (!map.has_rem(remote_entity)) continue;
            auto local_entity = map.remloc(remote_entity);
            if (!registry.valid(local_entity)) continue;

            if constexpr(std::is_empty_v<Component>) {
                registry.emplace<Component>(local_entity);
            } else {
                auto new_component = pair.second;
                merge<merge_type::created>(static_cast<Component *>(nullptr), new_component, ctx);
                registry.emplace<Component>(local_entity, new_component);
            }
        }
    }
};

template<typename Component>
struct destroyed_component_map: public component_map_base {
    entity_set entities;

    void insert(entt::entity entity) {
        entities.insert(entity);
    }

    void import(const registry_delta &, entt::registry &registry, entity_map &map) const override {
        for (auto remote_entity : entities) {
            if (!map.has_rem(remote_entity)) continue;
            auto local_entity = map.remloc(remote_entity);
            if (!registry.valid(local_entity)) continue;

            if (registry.has<Component>(local_entity)) {
                registry.remove<Component>(local_entity);
            }
        }
    }
};

struct island_topology {
    std::vector<size_t> count;
};

/**
 * Holds a set of changes made in one registry that can be imported into another
 * registry.
 */
class registry_delta {

    using map_of_component_map = std::unordered_map<entt::id_type, std::unique_ptr<component_map_base>>;

    void import_created_entities(entt::registry &, entity_map &) const;
    void import_destroyed_entities(entt::registry &, entity_map &) const;

    void import_updated_components(entt::registry &, entity_map &) const;
    void import_created_components(entt::registry &, entity_map &) const;
    void import_destroyed_components(entt::registry &, entity_map &) const;

    template<typename Component>
    void created(entt::entity entity, const Component &comp) {
        assure_components<Component, created_component_map>(&registry_delta::m_created_components).insert(entity, comp);
    }

    template<typename Component>
    void updated(entt::entity entity, const Component &comp) {
        assure_components<Component, updated_component_map>(&registry_delta::m_updated_components).insert(entity, comp);
    }

    template<typename Component>
    void destroyed(entt::entity entity) {
        assure_components<Component, destroyed_component_map>(&registry_delta::m_destroyed_components).insert(entity);
    }

    template<typename Component, template<typename> typename MapType>
    auto & assure_components(map_of_component_map registry_delta:: *member) {
        auto id = entt::type_index<Component>::value();
        using component_map_t = MapType<Component>;
        if ((this->*member).count(id) == 0) {
            (this->*member)[id].reset(new component_map_t);
        }
        return static_cast<component_map_t &>(*(this->*member).at(id));
    }

public:
    /**
     * Imports this delta into a registry by mapping the entities into the domain
     * of the target registry according to the provided `entity_map`.
     */
    void import(entt::registry &, entity_map &) const;

    bool empty() const;

    const auto created_entities() const { return m_created_entities; }

    template<typename Component>
    bool did_create(entt::entity entity) const {
        auto id = entt::type_index<Component>::value();
        if (m_created_components.count(id)) {
            auto &comp_map = static_cast<created_component_map<Component> &>(*m_created_components.at(id).get());
            return comp_map.pairs.count(entity) > 0;
        } else {
            return false;
        }
    }

    template<typename Component>
    bool did_destroy(entt::entity entity) const {
        auto id = entt::type_index<Component>::value();
        if (m_destroyed_components.count(id)) {
            auto &comp_map = static_cast<destroyed_component_map<Component> &>(*m_destroyed_components.at(id).get());
            return comp_map.entities.count(entity) > 0;
        } else {
            return false;
        }
    }

    friend class registry_delta_builder;

    double m_timestamp;

    island_topology m_island_topology;

private:
    entity_map m_entity_map;
    entity_set m_created_entities;
    entity_set m_destroyed_entities;

    map_of_component_map m_created_components;
    map_of_component_map m_updated_components;
    map_of_component_map m_destroyed_components;
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
        m_delta.created(entity, comp);
        (m_delta.created(entity, comps), ...);
    }

    template<typename Component>
    void created(entt::entity entity, entt::registry &registry) {
        if constexpr(entt::is_eto_eligible_v<Component>) {
            m_delta.created(entity, Component{});
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
     * Adds components to be updated by the delta.
     */
    template<typename... Component>
    void updated(entt::entity entity, Component &... comp) {
        (m_delta.updated(entity, comp), ...);
    }

    template<typename... Component>
    void updated(entt::entity entity, entt::registry &registry) {
        if constexpr(sizeof...(Component) <= 1) {
            if constexpr(std::conjunction_v<entt::is_eto_eligible<Component>...>) {
                (m_delta.updated(entity, Component{}), ...);
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
     * Marks components as deleted in this delta, or marks the entity as destroyed
     * if no component is specified.
     */
    template<typename... Component>
    void destroyed(entt::entity entity) {
        if constexpr(sizeof...(Component) == 0) {
            m_delta.m_destroyed_entities.insert(entity);
        } else {
            (m_delta.destroyed<Component>(entity), ...);
        }
    }

    template<typename... Component>
    void destroyed(entt::entity entity, [[maybe_unused]] std::tuple<Component...>) {
        destroyed<Component...>(entity);
    }

    template<typename... Component>
    void destroyed(entt::entity entity, entt::id_type id) {
        ((entt::type_index<Component>::value() == id ? destroyed<Component>(entity) : (void)0), ...);
    }

    template<typename... Component, typename It>
    void destroyed(entt::entity entity, It first, It last, std::tuple<Component...>) {
        for (auto it = first; it != last; ++it) {
            destroyed<Component...>(entity, *it);
        }
    }

    void topology(const island_topology &topo) {
        m_delta.m_island_topology = topo;
    }

    void clear() {
        m_delta = {};
    }

    bool empty() const {
        return m_delta.empty();
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