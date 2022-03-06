#ifndef EDYN_PARALLEL_ISLAND_DELTA_HPP
#define EDYN_PARALLEL_ISLAND_DELTA_HPP

#include <map>
#include <vector>
#include <entt/entity/fwd.hpp>
#include <entt/core/type_info.hpp>
#include "edyn/util/entity_map.hpp"
#include "edyn/parallel/entity_component_container.hpp"

namespace edyn {

/**
 * @brief Holds a set of changes made in one registry that can be imported
 * into another registry.
 */
class island_delta {

    using typed_component_container_map = std::map<entt::id_type, std::unique_ptr<entity_component_container_base>>;

    template<typename... Component>
    void reserve_created(size_t size);

    template<typename Component>
    void _reserve_created(size_t size);

    void import_created_entities(entt::registry &, entity_map &) const;
    void import_destroyed_entities(entt::registry &, entity_map &) const;

    void import_updated_components(entt::registry &, entity_map &) const;
    void import_created_components(entt::registry &, entity_map &) const;
    void import_destroyed_components(entt::registry &, entity_map &) const;

    template<typename Component, typename Func>
    void _created_for_each(Func func) const {
        auto id = entt::type_index<Component>();

        if (m_created_components.count(id) == 0) return;

        auto &created_ptr = m_created_components.at(id);
        using container_type = created_entity_component_container<Component>;
        auto *container = static_cast<const container_type *>(created_ptr.get());
        for (auto &pair : container->pairs) {
            func(pair.first, pair.second);
        }
    }

    template<typename Component, typename Func>
    void _updated_for_each(Func func) const {
        auto id = entt::type_index<Component>();

        if (m_updated_components.count(id) == 0) return;

        auto &updated_ptr = m_updated_components.at(id);
        using container_type = updated_entity_component_container<Component>;
        auto *container = static_cast<const container_type *>(updated_ptr.get());
        for (auto &pair : container->pairs) {
            func(pair.first, pair.second);
        }
    }

public:
    island_delta() = default;
    island_delta(island_delta &&) = default;

    // Explicitly delete copy constructor since this contains vectors of unique_ptrs.
    island_delta(const island_delta &) = delete;

    /**
     * Imports this delta into a registry by mapping the entities into the domain
     * of the target registry according to the provided `entity_map`.
     */
    void import(entt::registry &, entity_map &) const;

    bool empty() const;

    const auto created_entities() const { return m_created_entities; }

    template<typename... Component, typename Func>
    void created_for_each(Func func) const {
        (_created_for_each<Component>(func), ...);
    }

    template<typename... Component, typename Func>
    void created_for_each(std::tuple<Component...>, Func func) const {
        created_for_each<Component...>(func);
    }

    template<typename... Component, typename Func>
    void updated_for_each(Func func) const {
        (_updated_for_each<Component>(func), ...);
    }

    template<typename... Component, typename Func>
    void updated_for_each(std::tuple<Component...>, Func func) const {
        updated_for_each<Component...>(func);
    }

    friend class island_delta_builder;

    double m_timestamp;

private:
    entity_map m_entity_map;
    std::vector<entt::entity> m_created_entities;
    std::vector<entt::entity> m_destroyed_entities;

    typed_component_container_map m_created_components;
    typed_component_container_map m_updated_components;
    typed_component_container_map m_destroyed_components;
};

}

#endif // EDYN_PARALLEL_ISLAND_DELTA_HPP
