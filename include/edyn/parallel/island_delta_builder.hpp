#ifndef EDYN_PARALLEL_ISLAND_DELTA_BUILDER_HPP
#define EDYN_PARALLEL_ISLAND_DELTA_BUILDER_HPP

#include <tuple>
#include <memory>
#include <utility>
#include "edyn/parallel/entity_component_container.hpp"
#include "edyn/parallel/island_delta.hpp"

namespace edyn {

/**
 * @brief Provides the means to build a `island_delta`.
 */
class island_delta_builder {

    template<typename Component>
    void _created(entt::entity entity, const Component &component) {
        using container_type = created_entity_component_container<Component>;
        const auto index = entt::type_index<Component>::value();

        if (!(index < m_delta.m_created_components.size())) {
            m_delta.m_created_components.resize(index + 1);
        }

        if (auto &ptr = m_delta.m_created_components[index]; !ptr) {
            ptr.reset(new container_type());
        }

        auto *container = static_cast<container_type *>(m_delta.m_created_components[index].get());
        container->insert(entity, component);
    }

    template<typename Component>
    void _updated(entt::entity entity, const Component &component) {
        using container_type = updated_entity_component_container<Component>;
        const auto index = entt::type_index<Component>::value();

        if (!(index < m_delta.m_updated_components.size())) {
            m_delta.m_updated_components.resize(index + 1);
        }

        if (auto &ptr = m_delta.m_updated_components[index]; !ptr) {
            ptr.reset(new container_type());
        }

        auto *container = static_cast<container_type *>(m_delta.m_updated_components[index].get());
        container->insert(entity, component);
    }

    template<typename Component>
    void _destroyed(entt::entity entity) {
        using container_type = destroyed_entity_component_container<Component>;
        const auto index = entt::type_index<Component>::value();

        if (!(index < m_delta.m_destroyed_components.size())) {
            m_delta.m_destroyed_components.resize(index + 1);
        }

        if (auto &ptr = m_delta.m_destroyed_components[index]; !ptr) {
            ptr.reset(new container_type());
        }

        auto *container = static_cast<container_type *>(m_delta.m_destroyed_components[index].get());
        container->entities.push_back(entity);
    }

    template<typename Component>
    void _reserve_created(size_t size);

public:
    virtual ~island_delta_builder() {}

    /**
     * @brief Inserts a mapping into the current delta between a remote entity
     * and a local entity.
     * @param remote_entity Corresponding entity in the remote registry.
     * @param local_entity An entity in the local registry.
     */
    void insert_entity_mapping(entt::entity remote_entity, entt::entity local_entity);

    /**
     * @brief Marks the given entity as newly created.
     * @param entity The newly created entity.
     */
    void created(entt::entity);

    /**
     * @brief Adds the given components to the list of newly created components.
     * @tparam Component Pack of component types.
     * @param entity The entity the components have been assigned to.
     * @param comp A pack of component instances.
     */
    template<typename... Component>
    void created(entt::entity entity, const Component &... comp) {
        (_created(entity, comp), ...);
    }

    /**
     * @brief Fetches the requested components from the given registry and adds
     * them to the list of newly created components.
     * @tparam Component Pack of component types.
     * @param entity The entity the components have been assigned to.
     * @param registry The source registry.
     */
    template<typename Component>
    void created(entt::entity entity, entt::registry &registry) {
        if constexpr(entt::is_eto_eligible_v<Component>) {
            created(entity, Component{});
        } else {
            created(entity, registry.get<Component>(entity));
        }
    }

    /**
     * @brief Fetches the requested component by id (i.e. `entt::id_type`) from
     * the given registry and adds it to the list of newly created components.
     * @param entity The entity the component has been assigned to.
     * @param registry The source registry.
     * @param id A component id obtained using `entt::type_index`.
     */
    virtual void created(entt::entity entity, entt::registry &registry, entt::id_type id) = 0;

    /**
     * @brief Fetches the requested components by id (i.e. `entt::id_type`) from
     * the given registry and adds them to the list of newly created components.
     * @tparam It Type of input iterator.
     * @param entity The entity the components have been assigned to.
     * @param registry The source registry.
     * @param first An iterator to the first element of a range of component ids.
     * @param last An iterator past the last element of a range of component ids.
     */
    template<typename It>
    void created(entt::entity entity, entt::registry &registry, It first, It last) {
        for (auto it = first; it != last; ++it) {
            created(entity, registry, *it);
        }
    }

    /**
     * @brief Marks all registered component types that the given entity has as
     * newly created. Useful to be called for entities that have just been
     * constructed.
     * @param entity The entity the components have been assigned to.
     * @param registry The source registry.
     */
    virtual void created_all(entt::entity entity, entt::registry &registry) = 0;

    /**
     * @brief Adds the given components to the list of updated components.
     * @tparam Component Pack of component types.
     * @param entity The entity that owns the given components.
     * @param comp A pack of component instances.
     */
    template<typename... Component>
    void updated(entt::entity entity, const Component &... comp) {
        (_updated(entity, comp), ...);
    }

    /**
     * @brief Fetches the requested components from the given registry and adds
     * them to the list of updated components.
     * @tparam Component Pack of component types.
     * @param entity The entity that owns the given components.
     * @param registry The source registry.
     */
    template<typename... Component>
    void updated(entt::entity entity, entt::registry &registry) {
        if constexpr(sizeof...(Component) == 1) {
            if constexpr(std::conjunction_v<entt::is_eto_eligible<Component>...>) {
                (updated(entity, Component{}), ...);
            } else {
                (updated<Component>(entity, registry.get<Component>(entity)), ...);
            }
        } else {
            (updated<Component>(entity, registry), ...);
        }
    }

    /**
     * @brief Fetches the requested component by id (i.e. `entt::id_type`) from
     * the given registry and adds it to the list of updated components.
     * @tparam It Type of input iterator.
     * @param entity The entity the component has been assigned to.
     * @param registry The source registry.
     * @param id A component id obtained using `entt::type_index`.
     */
    virtual void updated(entt::entity entity, entt::registry &registry, entt::id_type id) = 0;

    /**
     * @brief Fetches the requested components by id (i.e. `entt::id_type`) from
     * the given registry and adds them to the list of updated components.
     * @tparam It Type of input iterator.
     * @param entity The entity the components have been assigned to.
     * @param registry The source registry.
     * @param first An iterator to the first element of a range of component ids.
     * @param last An iterator past the last element of a range of component ids.
     */
    template<typename It>
    void updated(entt::entity entity, entt::registry &registry, It first, It last) {
        for (auto it = first; it != last; ++it) {
            updated(entity, registry, *it);
        }
    }

    /**
     * @brief Marks all registered component types that the given entity has as
     * update.
     * @param entity The entity that owns the components.
     * @param registry The source registry.
     */
    virtual void updated_all(entt::entity entity, entt::registry &registry) = 0;

    /**
     * @brief Marks components as deleted or marks the entity as destroyed if no
     * component is specified.
     * @tparam Component Pack of component types.
     * @param entity The entity that owns the given components.
     */
    template<typename... Component>
    void destroyed(entt::entity entity) {
        if constexpr(sizeof...(Component) == 0) {
            m_delta.m_destroyed_entities.push_back(entity);
        } else {
            (_destroyed<Component>(entity), ...);
        }
    }

    /**
     * @brief Marks components as deleted by id (i.e. `entt::id_type`).
     * @param entity The entity that owned the deleted components.
     * @param id A component id obtained using `entt::type_index`.
     */
    virtual void destroyed(entt::entity entity, entt::id_type id) = 0;

    /**
     * @brief Marks components as deleted by id (i.e. `entt::id_type`).
     * @tparam It Type of input iterator.
     * @param entity The entity that owned the deleted components.
     * @param first An iterator to the first element of a range of component ids.
     * @param last An iterator past the last element of a range of component ids.
     */
    template<typename It>
    void destroyed(entt::entity entity, It first, It last) {
        for (auto it = first; it != last; ++it) {
            destroyed(entity, *it);
        }
    }

    /**
     * @brief Reserves the given amount of entries in the vector for created components
     * of the given type for more performant insertion. If no component is provided
     * it reserves the given amount of entries in the vector for created entities.
     * @tparam Component The type to be reserved.
     * @param size The number of entries to be reserved.
     */
    template<typename... Component>
    void reserve_created(size_t size);

    /**
     * @brief Returns whether the current delta contains no changes.
     * @return Whether the current delta contains no changes.
     */
    bool empty() const;

    /**
     * @brief Returns whether an island worker should be woken up to process the current
     * delta. Waking up is usually not necessary if the delta does not include
     * changes that would be applied to the registry, e.g. if it only contains
     * entity mappings.
     * @return Whether destination will have to be woken up to process the current delta.
     */
    bool needs_wakeup() const;

    island_delta finish() {
        // Move the contents of `m_delta` into the returned object, effectively
        // clearing out the contents of `m_delta` making it ready for the next
        // set of updates.
        return std::move(m_delta);
    }

private:
    island_delta m_delta;
};

template<typename... Component>
void island_delta_builder::reserve_created(size_t size) {
    if constexpr(sizeof...(Component) == 0) {
        m_delta.m_created_entities.reserve(size);
    } else {
        (_reserve_created<Component>(size), ...);
    }
}

template<typename Component>
void island_delta_builder::_reserve_created(size_t size) {
    using container_type = created_entity_component_container<Component>;
    const auto index = entt::type_index<Component>::value();

    if (!(index < m_delta.m_created_components.size())) {
        m_delta.m_created_components.resize(index + 1);
    }

    if (auto &ptr = m_delta.m_created_components[index]; !ptr) {
        ptr.reset(new container_type());
    }

    auto *container = static_cast<container_type *>(m_delta.m_created_components[index].get());
    container->reserve(size);
}

/**
 * @brief Implementation of `island_delta_builder` which allows a list of
 * support components to be specified.
 * 
 * @note
 * When users find the need to add extra logic to the physics simulation, they'll
 * need to have their custom components be shared between island coordinator and 
 * island worker so that their custom system update functions can work on these
 * components (functions assigned via `edyn::set_external_system_pre_step`, etc).
 * This class provides implementations of the functions that update the 
 * `island_delta` by component id, which is required for functionalities such
 * as marking all components as created/updated and marking components as dirty.
 * 
 * @tparam Component Pack of supported component types.
 */
template<typename... Component>
class island_delta_builder_impl: public island_delta_builder {
public:
    island_delta_builder_impl([[maybe_unused]] std::tuple<Component...>)
    {}

    void created(entt::entity entity, entt::registry &registry, entt::id_type id) override {
        ((entt::type_index<Component>::value() == id ? 
            island_delta_builder::created<Component>(entity, registry) : (void)0), ...);
    }

    void created_all(entt::entity entity, entt::registry &registry) override {
        ((registry.has<Component>(entity) ? 
            island_delta_builder::created<Component>(entity, registry) : (void)0), ...);
    }

    void updated(entt::entity entity, entt::registry &registry, entt::id_type id) override {
        ((entt::type_index<Component>::value() == id ? 
            island_delta_builder::updated<Component>(entity, registry) : (void)0), ...);
    }

    void updated_all(entt::entity entity, entt::registry &registry) override {
        ((registry.has<Component>(entity) ? 
            island_delta_builder::updated<Component>(entity, registry) : (void)0), ...);
    }

    void destroyed(entt::entity entity, entt::id_type id) override {
        ((entt::type_index<Component>::value() == id ? 
            island_delta_builder::destroyed<Component>(entity) : (void)0), ...);
    }
};

/**
 * @brief Function type of a factory function that creates instances of a 
 * registry delta builder implementation.
 */
using make_island_delta_builder_func_t = std::unique_ptr<island_delta_builder>(*)();

/**
 * @brief Pointer to a factory function that makes new delta builders.
 * 
 * The default function returns a delta builder configured with all default 
 * shared components (`edyn::shared_components`) but it can be replaced by
 * a function that returns a builder which additionally handles external
 * components set by the user.
 */
extern make_island_delta_builder_func_t g_make_island_delta_builder;

/**
 * @brief Creates a new delta builder.
 * 
 * Returns a delta builder implementation that supports handling all shared 
 * component types plus any external component set by the user.
 * 
 * @return Safe pointer to an instance of a delta builder implementation.
 */
std::unique_ptr<island_delta_builder> make_island_delta_builder();

/**
 * @brief Registers external components to be shared between island coordinator
 * and island workers. 
 * @tparam Component External component types.
 */
template<typename... Component>
void register_external_components() {
    g_make_island_delta_builder = [] () {
        auto external = std::tuple<Component...>{};
        auto all_components = std::tuple_cat(edyn::shared_components{}, external);
        return std::unique_ptr<edyn::island_delta_builder>(
            new edyn::island_delta_builder_impl(all_components));
    };
}

/**
 * @brief Removes registered external components and resets to defaults.
 */
void remove_external_components();

}

#endif // EDYN_PARALLEL_ISLAND_DELTA_BUILDER_HPP
