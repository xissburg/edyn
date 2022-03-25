#ifndef EDYN_NETWORKING_UTIL_CLIENT_SNAPSHOT_EXPORTER_HPP
#define EDYN_NETWORKING_UTIL_CLIENT_SNAPSHOT_EXPORTER_HPP

#include <entt/entity/fwd.hpp>
#include "edyn/networking/util/registry_snapshot.hpp"

namespace edyn {

class client_snapshot_exporter {
public:
    virtual ~client_snapshot_exporter() = default;

    // Write all networked entities and components into a snapshot.
    virtual void export_all(const entt::registry &registry, registry_snapshot &snap) = 0;

    // Write all transient entities and components into a snapshot.
    virtual void export_transient(const entt::registry &registry, registry_snapshot &snap) = 0;

    // Write all input entities and components into a snapshot.
    virtual void export_input(const entt::registry &registry, registry_snapshot &snap) = 0;

    // Write all transient entities and components which are also input into a snapshot.
    virtual void export_transient_input(const entt::registry &registry, registry_snapshot &snap) = 0;

    // Write a single entity and component by type id into a snapshot.
    virtual void export_by_type_id(const entt::registry &registry,
                                   entt::entity entity, entt::id_type id,
                                   registry_snapshot &snap) = 0;

    // Check whether an entity contains one or more transient components.
    virtual bool contains_transient(const entt::registry &registry, entt::entity entity) const = 0;

    // Check whether an entity contains one or more transient components which
    // are also input.
    virtual bool contains_transient_input(const entt::registry &registry, entt::entity entity) const = 0;

    // Check whether a type is a transient component by id.
    virtual bool is_transient(entt::id_type id) const = 0;
};

template<typename... Components>
class client_snapshot_exporter_impl : public client_snapshot_exporter {

    template<unsigned... ComponentIndex>
    void export_by_type_id(const entt::registry &registry,
                           entt::entity entity, entt::id_type id,
                           registry_snapshot &snap,
                           std::integer_sequence<unsigned, ComponentIndex...>) {
        ((entt::type_id<Components>().seq() == id ?
            internal::snapshot_insert_entity<Components>(registry, entity, snap, ComponentIndex) : void(0)), ...);
    }

    using insert_entity_components_func_t = void(const entt::registry &, registry_snapshot &);
    insert_entity_components_func_t *m_insert_transient_entity_components_func;
    insert_entity_components_func_t *m_insert_input_entity_components_func;
    insert_entity_components_func_t *m_insert_transient_input_entity_components_func;

    using contains_transient_func_t = bool(const entt::registry &, entt::entity);
    contains_transient_func_t *m_contains_transient;
    contains_transient_func_t *m_contains_transient_input;

public:
    template<typename... Transient, typename... Input>
    client_snapshot_exporter_impl(std::tuple<Components...>, std::tuple<Transient...>, std::tuple<Input...>) {
        static_assert((!std::is_empty_v<Input> && ...));

        m_insert_transient_entity_components_func = [] (const entt::registry &registry, registry_snapshot &snap) {
            const std::tuple<Components...> components;
            internal::snapshot_insert_select_entity_components<Transient...>(registry, snap, components);
        };

        m_insert_input_entity_components_func = [] (const entt::registry &registry, registry_snapshot &snap) {
            const std::tuple<Components...> components;
            internal::snapshot_insert_select_entity_components<Input...>(registry, snap, components);
        };

        m_insert_transient_input_entity_components_func = [] (const entt::registry &registry, registry_snapshot &snap) {
            const std::tuple<Components...> components;
            ((has_type<Transient, std::tuple<Input...>>::value ?
                internal::snapshot_insert_select_entity_component<Transient>(registry, snap, components) : (void)0), ...);
        };

        m_contains_transient = [] (const entt::registry &registry, entt::entity entity) {
            return registry.any_of<Transient...>(entity);
        };

        m_contains_transient_input = [] (const entt::registry &registry, entt::entity entity) {
            return ((has_type<Transient, std::tuple<Input...>>::value && registry.any_of<Transient>(entity)) || ...);
        };

        ((m_is_transient_component[entt::type_id<Components>().seq()] = has_type<Components, std::tuple<Transient...>>::value), ...);
    }

    void export_all(const entt::registry &registry, registry_snapshot &snap) override {
        const std::tuple<Components...> components;
        internal::snapshot_insert_entity_components_all(registry, snap, components,
                                                        std::make_index_sequence<sizeof...(Components)>{});
    }

    void export_transient(const entt::registry &registry, registry_snapshot &snap) override {
        (*m_insert_transient_entity_components_func)(registry, snap);
    }

    void export_input(const entt::registry &registry, registry_snapshot &snap) override {
        (*m_insert_input_entity_components_func)(registry, snap);
    }

    void export_transient_input(const entt::registry &registry, registry_snapshot &snap) override {
        (*m_insert_transient_input_entity_components_func)(registry, snap);
    }

    void export_by_type_id(const entt::registry &registry,
                           entt::entity entity, entt::id_type id,
                           registry_snapshot &snap) override {
        export_by_type_id(registry, entity, id, snap, std::make_integer_sequence<unsigned, sizeof...(Components)>{});
    }

    bool contains_transient(const entt::registry &registry, entt::entity entity) const override {
        return (*m_contains_transient)(registry, entity);
    }

    bool contains_transient_input(const entt::registry &registry, entt::entity entity) const override {
        return (*m_contains_transient_input)(registry, entity);
    }

    bool is_transient(entt::id_type id) const override {
        if (m_is_transient_component.count(id)) {
            return m_is_transient_component.at(id);
        }

        return false;
    }

private:
    std::map<entt::id_type, bool> m_is_transient_component;
};

}

#endif // EDYN_NETWORKING_UTIL_CLIENT_SNAPSHOT_EXPORTER_HPP
