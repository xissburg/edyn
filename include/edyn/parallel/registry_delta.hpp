#ifndef EDYN_PARALLEL_REGISTRY_DELTA_HPP
#define EDYN_PARALLEL_REGISTRY_DELTA_HPP

#include <vector>
#include <entt/fwd.hpp>
#include <unordered_map>
#include <entt/core/type_info.hpp>
#include "edyn/comp/shared_comp.hpp"
#include "edyn/util/entity_map.hpp"
#include "edyn/parallel/delta_component_map.hpp"

namespace edyn {

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

    void clear();

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
    std::vector<entt::entity> m_created_entities;
    std::vector<entt::entity> m_destroyed_entities;

    map_of_component_map m_created_components;
    map_of_component_map m_updated_components;
    map_of_component_map m_destroyed_components;
};

}

#endif // EDYN_PARALLEL_REGISTRY_DELTA_HPP
