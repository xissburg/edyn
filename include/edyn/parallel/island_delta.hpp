#ifndef EDYN_PARALLEL_ISLAND_DELTA_HPP
#define EDYN_PARALLEL_ISLAND_DELTA_HPP

#include <vector>
#include <entt/fwd.hpp>
#include <entt/core/type_info.hpp>
#include "edyn/comp/shared_comp.hpp"
#include "edyn/util/entity_map.hpp"
#include "edyn/parallel/entity_component_map.hpp"
#include "edyn/parallel/entity_component_container.hpp"

namespace edyn {

struct island_topology {
    std::vector<size_t> count;
};

/**
 * @brief Holds a set of changes made in one registry that can be imported
 * into another registry.
 */
class island_delta {

    using typed_component_container_vector = std::vector<std::pair<entt::id_type, std::unique_ptr<entity_component_container_base>>>;

    template<typename Component>
    void prepare_created() {
        auto id = entt::type_index<Component>::value();
        m_created_components.emplace_back(id, std::make_unique<created_entity_component_container<Component>>());
    }

    template<typename Component>
    void prepare_updated() {
        auto id = entt::type_index<Component>::value();
        m_updated_components.emplace_back(id, std::make_unique<updated_entity_component_container<Component>>());
    }

    template<typename Component>
    void prepare_destroyed() {
        auto id = entt::type_index<Component>::value();
        m_destroyed_components.emplace_back(id, std::make_unique<destroyed_entity_component_container<Component>>());
    }

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

    friend class island_delta_builder;

    double m_timestamp;

    island_topology m_island_topology;

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
