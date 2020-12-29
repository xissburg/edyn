#ifndef EDYN_PARALLEL_REGISTRY_DELTA_BUILDER_HPP
#define EDYN_PARALLEL_REGISTRY_DELTA_BUILDER_HPP

#include <tuple>
#include <memory>
#include <utility>
#include "edyn/parallel/registry_delta.hpp"

namespace edyn {

class registry_delta_builder {
public:
    registry_delta_builder(entity_map &map)
        : m_entity_map(&map)
    {}

    void insert_entity_mapping(entt::entity);

    void created(entt::entity entity) {
        m_delta.m_created_entities.insert(entity);
    }

    template<typename... Component>
    void created(entt::entity entity, const Component &... comp) {
        (m_delta.created(entity, comp), ...);
    }

    template<typename Component>
    void created(entt::entity entity, entt::registry &registry) {
        if constexpr(entt::is_eto_eligible_v<Component>) {
            m_delta.created(entity, Component{});
        } else {
            created<Component>(entity, registry.get<Component>(entity));
        }
    }

    virtual void created(entt::entity entity, entt::registry &registry, entt::id_type id) = 0;

    template<typename It>
    void created(entt::entity entity, entt::registry &registry, It first, It last) {
        for (auto it = first; it != last; ++it) {
            created(entity, registry, *it);
        }
    }

    virtual void created_all(entt::entity entity, entt::registry &registry) = 0;

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

    virtual void updated(entt::entity entity, entt::registry &registry, entt::id_type id) = 0;

    template<typename It>
    void updated(entt::entity entity, entt::registry &registry, It first, It last) {
        for (auto it = first; it != last; ++it) {
            updated(entity, registry, *it);
        }
    }

    virtual void updated_all(entt::entity entity, entt::registry &registry) = 0;

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

    virtual void destroyed(entt::entity entity, entt::id_type id) = 0;

    template<typename It>
    void destroyed(entt::entity entity, It first, It last) {
        for (auto it = first; it != last; ++it) {
            destroyed(entity, *it);
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

template<typename... Component>
class registry_delta_builder_impl: public registry_delta_builder {
public:
    registry_delta_builder_impl(entity_map &map, [[maybe_unused]] std::tuple<Component...>)
        : registry_delta_builder(map)
    {}

    void created(entt::entity entity, entt::registry &registry, entt::id_type id) override {
        ((entt::type_index<Component>::value() == id ? 
            registry_delta_builder::created<Component>(entity, registry) : (void)0), ...);
    }

    void created_all(entt::entity entity, entt::registry &registry) override {
        ((registry.has<Component>(entity) ? 
            registry_delta_builder::created<Component>(entity, registry) : (void)0), ...);
    }

    void updated(entt::entity entity, entt::registry &registry, entt::id_type id) override {
        ((entt::type_index<Component>::value() == id ? 
            registry_delta_builder::updated<Component>(entity, registry) : (void)0), ...);
    }

    void updated_all(entt::entity entity, entt::registry &registry) override {
        ((registry.has<Component>(entity) ? 
            registry_delta_builder::updated<Component>(entity, registry) : (void)0), ...);
    }

    void destroyed(entt::entity entity, entt::id_type id) override {
        ((entt::type_index<Component>::value() == id ? 
            registry_delta_builder::destroyed<Component>(entity) : (void)0), ...);
    }
};

using make_registry_delta_builder_func_t = std::unique_ptr<registry_delta_builder>(*)(entity_map &);
extern make_registry_delta_builder_func_t g_make_registry_delta_builder;

std::unique_ptr<registry_delta_builder> make_registry_delta_builder(entity_map &);

}

#endif // EDYN_PARALLEL_REGISTRY_DELTA_BUILDER_HPP