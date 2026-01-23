#ifndef EDYN_REPLICATION_REGISTRY_OPERATION_OBSERVER_HPP
#define EDYN_REPLICATION_REGISTRY_OPERATION_OBSERVER_HPP

#include "edyn/collision/contact_point.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/replication/registry_operation_builder.hpp"
#include <bits/utility.h>
#include <entt/entity/fwd.hpp>
#include <entt/signal/sigh.hpp>

namespace edyn {

class registry_operation_observer {
public:
    registry_operation_observer(registry_operation_builder &builder)
        : m_builder(&builder)
        , m_active(true)
    {
    }

    virtual ~registry_operation_observer() {}

    void observe(entt::entity entity) {
        m_observed_entities.push(entity);

        if (m_active) {
            m_builder->create(entity);
            m_builder->emplace_all(entity);
        }
    }

    void unobserve(entt::entity entity) {
        m_observed_entities.erase(entity);

        if (m_active) {
            m_builder->destroy(entity);
        }
    }

    void set_active(bool active) {
        m_active = active;
    }

protected:
    registry_operation_builder *m_builder;
    entt::sparse_set m_observed_entities;
    std::vector<entt::scoped_connection> m_connections;
    bool m_active;
};

template<typename... Components>
class registry_operation_observer_impl : public registry_operation_observer {
    template<typename Component>
    void on_construct(entt::registry &registry, entt::entity entity) {
        if (m_active && m_observed_entities.contains(entity)) {
            m_builder->emplace<Component>(entity);
        }
    }

    template<typename Component>
    void on_update(entt::registry &registry, entt::entity entity) {
        if (m_active && m_observed_entities.contains(entity)) {
            m_builder->replace<Component>(entity);
        }
    }

    template<typename Component>
    void on_destroy(entt::registry &registry, entt::entity entity) {
        if (m_active && m_observed_entities.contains(entity)) {
            m_builder->remove<Component>(entity);
        }
    }

    template<typename Component, unsigned StorageIndex>
    void on_construct_storage(entt::registry &registry, entt::entity entity) {
        if (m_active && m_observed_entities.contains(entity)) {
            constexpr auto storage_name = contact_point_storage_names[StorageIndex];
            m_builder->emplace_storage<Component>(storage_name, entity);
        }
    }

    template<typename Component, unsigned StorageIndex>
    void on_update_storage(entt::registry &registry, entt::entity entity) {
        if (m_active && m_observed_entities.contains(entity)) {
            constexpr auto storage_name = contact_point_storage_names[StorageIndex];
            m_builder->replace_storage<Component>(storage_name, entity);
        }
    }

    template<typename Component, unsigned StorageIndex>
    void on_destroy_storage(entt::registry &registry, entt::entity entity) {
        if (m_active && m_observed_entities.contains(entity)) {
            constexpr auto storage_name = contact_point_storage_names[StorageIndex];
            m_builder->remove_storage<Component>(storage_name, entity);
        }
    }

    template<unsigned StorageIndex>
    void observe_storage(entt::registry &registry) {
        constexpr auto storage_name = contact_point_storage_names[StorageIndex];
        m_connections.push_back(registry.storage<contact_point>(storage_name).on_construct().template connect<&registry_operation_observer_impl<Components...>::template on_construct_storage<contact_point, StorageIndex>>(*this));
        m_connections.push_back(registry.storage<contact_point>(storage_name).on_update().template connect<&registry_operation_observer_impl<Components...>::template on_update_storage<contact_point, StorageIndex>>(*this));
        m_connections.push_back(registry.storage<contact_point>(storage_name).on_destroy().template connect<&registry_operation_observer_impl<Components...>::template on_destroy_storage<contact_point, StorageIndex>>(*this));
    }

    template<unsigned... StorageIndex>
    void observe_storages(entt::registry &registry, std::integer_sequence<unsigned, StorageIndex...>) {
        (observe_storage<StorageIndex>(registry),...);
    }

public:
    registry_operation_observer_impl(registry_operation_builder &builder, [[maybe_unused]] std::tuple<Components...>)
        : registry_operation_observer(builder)
    {
        auto &registry = builder.get_registry();
        (m_connections.push_back(registry.on_construct<Components>().template connect<&registry_operation_observer_impl<Components...>::template on_construct<Components>>(*this)), ...);
        (m_connections.push_back(registry.on_update<Components>().template connect<&registry_operation_observer_impl<Components...>::template on_update<Components>>(*this)), ...);
        (m_connections.push_back(registry.on_destroy<Components>().template connect<&registry_operation_observer_impl<Components...>::template on_destroy<Components>>(*this)), ...);

        observe_storages(registry, std::make_integer_sequence<unsigned, contact_point_storage_names.size()>{});
    }

    virtual ~registry_operation_observer_impl() {}
};

}

#endif // EDYN_REPLICATION_REGISTRY_OPERATION_OBSERVER_HPP
