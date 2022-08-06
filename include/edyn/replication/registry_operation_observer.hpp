#ifndef EDYN_REPLICATION_REGISTRY_OPERATION_OBSERVER_HPP
#define EDYN_REPLICATION_REGISTRY_OPERATION_OBSERVER_HPP

#include "edyn/replication/registry_operation_builder.hpp"
#include <entt/signal/sigh.hpp>

namespace edyn {

class registry_operation_observer {
public:
    registry_operation_observer(registry_operation_builder &builder)
        : m_builder(&builder)
    {}

    virtual ~registry_operation_observer() {}

protected:
    registry_operation_builder *m_builder;
};

template<typename... Components>
class registry_operation_observer_impl : public registry_operation_observer {
    template<typename Component>
    void on_construct(entt::registry &registry, entt::entity entity) {
        m_builder->emplace<Component>(entity);
    }

    template<typename Component>
    void on_update(entt::registry &registry, entt::entity entity) {
        m_builder->replace<Component>(entity);
    }

    template<typename Component>
    void on_destroy(entt::registry &registry, entt::entity entity) {
        m_builder->remove<Component>(entity);
    }

public:
    registry_operation_observer_impl(registry_operation_builder &builder, [[maybe_unused]] std::tuple<Components...>)
        : registry_operation_observer(builder)
    {
        auto &registry = builder.get_registry();
        (m_connections.push_back(registry.on_construct<Components>().template connect<&registry_operation_observer_impl<Components...>::template on_construct<Components>>(*this)), ...);
        (m_connections.push_back(registry.on_update<Components>().template connect<&registry_operation_observer_impl<Components...>::template on_update<Components>>(*this)), ...);
        (m_connections.push_back(registry.on_destroy<Components>().template connect<&registry_operation_observer_impl<Components...>::template on_destroy<Components>>(*this)), ...);
    }

    virtual ~registry_operation_observer_impl() {}

private:
    std::vector<entt::scoped_connection> m_connections;
};

}

#endif // EDYN_REPLICATION_REGISTRY_OPERATION_OBSERVER_HPP
