#ifndef EDYN_PARALLEL_ISLAND_DELTA_HPP
#define EDYN_PARALLEL_ISLAND_DELTA_HPP

#include <vector>
#include <entt/fwd.hpp>
#include <entt/core/type_info.hpp>
#include "edyn/comp/shared_comp.hpp"
#include "edyn/util/entity_map.hpp"
#include "edyn/parallel/entity_component_container.hpp"

namespace edyn {

/**
 * @brief Holds a set of changes made in one registry that can be imported
 * into another registry.
 */
class island_delta {

    using typed_component_container_vector = std::vector<std::unique_ptr<entity_component_container_base>>;

    template<typename... Component>
    void reserve_created(size_t size);
    
    template<typename Component>
    void _reserve_created(size_t size);

    void import_created_entities(entt::registry &, entity_map &) const;
    void import_destroyed_entities(entt::registry &, entity_map &) const;

    void import_updated_components(entt::registry &, entity_map &) const;
    void import_created_components(entt::registry &, entity_map &) const;
    void import_destroyed_components(entt::registry &, entity_map &) const;

public:
    /**
     * Imports this delta into a registry by mapping the entities into the domain
     * of the target registry according to the provided `entity_map`.
     */
    void import(entt::registry &, entity_map &) const;

    bool empty() const;

    const auto created_entities() const { return m_created_entities; }

    template<typename Component, typename Func>
    void created_for_each(Func func) const {
        auto index = entt::type_index<Component>::value();
        if (!(index < m_created_components.size())) return;

        if (auto &created_ptr = m_created_components[index]; created_ptr) {
            using container_type = created_entity_component_container<Component>;
            auto *container = static_cast<const container_type *>(created_ptr.get());
            for (auto &pair : container->pairs) {
                func(pair.first, pair.second);
            }
        }
    }

    template<typename Component, typename Func>
    void updated_for_each(Func func) const {
        auto index = entt::type_index<Component>::value();
        if (!(index < m_updated_components.size())) return;

        if (auto &updated_ptr = m_updated_components[index]; updated_ptr) {
            using container_type = updated_entity_component_container<Component>;
            auto *container = static_cast<const container_type *>(updated_ptr.get());
            for (auto &pair : container->pairs) {
                func(pair.first, pair.second);
            }
        }
    }

    friend class island_delta_builder;

    double m_timestamp;

private:
    entity_map m_entity_map;
    std::vector<entt::entity> m_created_entities;
    std::vector<entt::entity> m_destroyed_entities;

    typed_component_container_vector m_created_components;
    typed_component_container_vector m_updated_components;
    typed_component_container_vector m_destroyed_components;
};

}

#endif // EDYN_PARALLEL_ISLAND_DELTA_HPP
