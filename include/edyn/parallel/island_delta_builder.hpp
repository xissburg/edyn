#ifndef EDYN_PARALLEL_ISLAND_DELTA_BUILDER_HPP
#define EDYN_PARALLEL_ISLAND_DELTA_BUILDER_HPP

#include <entt/core/type_info.hpp>
#include <tuple>
#include <memory>
#include <utility>
#include <entt/entity/fwd.hpp>
#include "edyn/comp/shared_comp.hpp"
#include "edyn/parallel/island_delta.hpp"
#include "edyn/parallel/entity_component_container.hpp"

namespace edyn {

/**
 * @brief Provides the means to build an `island_delta`.
 */
class island_delta_builder {

    template<typename Component>
    void _created(entt::entity entity, const Component &component) {
        using container_type = created_entity_component_container<Component>;
        const auto id = entt::type_index<Component>();

        if (m_delta.m_created_components.count(id) == 0) {
            m_delta.m_created_components[id].reset(new container_type());
        }

        auto *container = static_cast<container_type *>(m_delta.m_created_components.at(id).get());
        container->insert(entity, component);
    }

    // Only enable updates for non-empty components since it does not make
    // sense to update empty components because they have no data.
    template<typename Component, std::enable_if_t<!std::is_empty_v<Component>, bool> = true>
    void _updated(entt::entity entity, const Component &component) {
        using container_type = updated_entity_component_container<Component>;
        const auto id = entt::type_index<Component>();

        if (m_delta.m_updated_components.count(id) == 0) {
            m_delta.m_updated_components[id].reset(new container_type());
        }

        auto *container = static_cast<container_type *>(m_delta.m_updated_components.at(id).get());
        container->insert(entity, component);
    }

    template<typename Component>
    void _destroyed(entt::entity entity) {
        using container_type = destroyed_entity_component_container<Component>;
        const auto id = entt::type_index<Component>();

        if (m_delta.m_destroyed_components.count(id) == 0) {
            m_delta.m_destroyed_components[id].reset(new container_type());
        }

        auto *container = static_cast<container_type *>(m_delta.m_destroyed_components.at(id).get());
        container->entities.push_back(entity);
    }

    template<typename Component>
    void _reserve_created(size_t size);

public:
    island_delta_builder() = default;
    island_delta_builder(island_delta_builder &&) = default;

    // Explicitly delete copy constructor because `island_delta` also deletes it.
    island_delta_builder(const island_delta_builder &) = delete;

    virtual ~island_delta_builder() {}

    /**
     * @brief Inserts a mapping into the current delta between a remote entity
     * and a local entity.
     * @param remote_entity Corresponding entity in the remote registry.
     * @param local_entity An entity in the local registry.
     */
    void insert_entity_mapping(entt::entity remote_entity, entt::entity local_entity);

    /**
     * @brief Check whether a remote entity is present in the current snapshot's
     * entity mapping.
     * @param remote_entity Entity in remote registry.
     * @return Whether an remote entity was already inserted.
     */
    bool has_rem(entt::entity remote_entity) const;

    /**
     * @brief Check whether a local entity is present in the current snapshot's
     * entity mapping.
     * @param remote_entity Entity in local registry.
     * @return Whether an local entity was already inserted.
     */
    bool has_loc(entt::entity local_entity) const;

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
        if constexpr(std::is_empty_v<Component>) {
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
            // Empty components cannot be updated.
            if constexpr(!std::conjunction_v<std::is_empty<Component>...>) {
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
    const auto id = entt::type_index<Component>();

    if (m_delta.m_created_components.count(id) == 0) {
        m_delta.m_created_components[id].reset(new container_type());
    }

    auto *container = static_cast<container_type *>(m_delta.m_created_components.at(id).get());
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
    island_delta_builder_impl([[maybe_unused]] std::tuple<Component...>) {

    }

    void created(entt::entity entity, entt::registry &registry, entt::id_type id) override {
        ((entt::type_index<Component>() == id ?
            island_delta_builder::created<Component>(entity, registry) : (void)0), ...);
    }

    void created_all(entt::entity entity, entt::registry &registry) override {
        ((registry.any_of<Component>(entity) ?
            island_delta_builder::created<Component>(entity, registry) : (void)0), ...);
    }

    void updated(entt::entity entity, entt::registry &registry, entt::id_type id) override {
        ((entt::type_index<Component>() == id ?
            island_delta_builder::updated<Component>(entity, registry) : (void)0), ...);
    }

    void updated_all(entt::entity entity, entt::registry &registry) override {
        ((registry.any_of<Component>(entity) ?
            island_delta_builder::updated<Component>(entity, registry) : (void)0), ...);
    }

    void destroyed(entt::entity entity, entt::id_type id) override {
        ((entt::type_index<Component>() == id ?
            island_delta_builder::destroyed<Component>(entity) : (void)0), ...);
    }
};

}

#endif // EDYN_PARALLEL_ISLAND_DELTA_BUILDER_HPP
