#ifndef EDYN_SERIALIZATION_REGISTRY_S11N_HPP
#define EDYN_SERIALIZATION_REGISTRY_S11N_HPP

#include <unordered_map>
#include <cstdint>
#include <entt/entt.hpp>
#include "edyn/util/tuple.hpp"
#include "edyn/util/entity_map.hpp"

namespace edyn {

/**
 * @brief The `registry_snapshot_writer` stores a reference to a registry and a
 * list of entities and component types to be serialized. It must be serialized
 * with an output archive. The specified entities and components are written to
 * the archive.
 * @tparam Component The list of component types that can be serialized using
 *      this writer.
 */
template<typename... Component>
class registry_snapshot_writer {
protected:
    template<size_t... Indexes>
    bool has(entt::entity entity, size_t comp_id, std::index_sequence<Indexes...>) {
        return ((comp_id == Indexes ? m_registry->has<std::tuple_element_t<Indexes, tuple_type>>(entity) : false) || ...);
    }

    bool has(entt::entity entity, size_t comp_id) {
        return has(entity, comp_id, std::make_index_sequence<sizeof...(Component)>{});
    }

    template<typename Func, size_t... Indexes>
    void try_visit(entt::entity entity, size_t comp_id, Func f, std::index_sequence<Indexes...>) {
        ((comp_id == Indexes ? f(m_registry->try_get<std::tuple_element_t<Indexes, tuple_type>>(entity)) : (void)0), ...);
    }

    template<typename Func>
    void try_visit(entt::entity entity, size_t comp_id, Func f) {
        try_visit(entity, comp_id, f, std::make_index_sequence<sizeof...(Component)>{});
    }

public:
    using tuple_type = std::tuple<Component...>;

    registry_snapshot_writer(entt::registry &reg) 
        : m_registry(&reg)
    {}

    /**
     * @brief Specifies which entities and components should be serialized.
     * @tparam Comp Component types to be serialized.
     * @tparam It Iterator type.
     * @param first Iterator in the beginning of the list of entities.
     * @param last Iterator in the end of the list of entities.
     */
    template<typename... Comp, typename It>
    void component(It first, It last) {
        m_entities.resize(std::distance(first, last));
        std::copy(first, last, m_entities.begin());
        (m_component_ids.push_back(index_of_v<Comp, Component...>), ...);
    }

    template<typename Archive, typename... Ts>
    friend void serialize(Archive &, registry_snapshot_writer<Ts...> &);

protected:
    entt::registry *m_registry;
    std::vector<entt::entity> m_entities;
    std::vector<size_t> m_component_ids;
};

/**
 * @brief Writes the selected registry data into the archive.
 * @param archive An output archive.
 * @param snapshot The snapshot writer.
 */
template<typename Archive, typename... Component>
void serialize(Archive &archive, registry_snapshot_writer<Component...> &snapshot) {
    static_assert(Archive::is_output::value, "Output archive expected.");
    
    archive(snapshot.m_entities.size());

    for (auto entity : snapshot.m_entities) {
        archive(entity);

        size_t count = 0;
        for (auto comp_id : snapshot.m_component_ids) {
            if (snapshot.has(entity, comp_id)) {
                ++count;
            }
        }
        archive(count);

        for (auto comp_id : snapshot.m_component_ids) {
            snapshot.try_visit(entity, comp_id, [&archive, &comp_id] (auto *comp) {
                if (comp) {
                    archive(comp_id);
                    archive(*comp);
                }
            });
        }
    }
}

//-----------------------------------------------------------------------------

/**
 * @brief The `registry_snapshot_reader` stores a reference to a registry and when 
 * serialized (using an input archive) it reads the entities and components 
 * from the data buffer and assigns or replaces the components found in the
 * buffer into the associated target registry.
 */
template<typename... Component>
class registry_snapshot_reader {
protected:
    template<typename Func, size_t... Indexes>
    void visit(entt::entity entity, size_t comp_id, Func f, std::index_sequence<Indexes...>) {
        ((comp_id == Indexes ? f(std::tuple_element_t<Indexes, tuple_type>{}) : (void)0), ...);
    }

    template<typename Func>
    void visit(entt::entity entity, size_t comp_id, Func f) {
        visit(entity, comp_id, f, std::make_index_sequence<sizeof...(Component)>{});
    }

public:
    using tuple_type = std::tuple<Component...>;

    registry_snapshot_reader(entt::registry &reg) 
        : m_registry(&reg)
    {}

    template<typename Archive, typename... Ts>
    friend void serialize(Archive &, registry_snapshot_reader<Ts...> &);

protected:
    entt::registry *m_registry;
    //std::unordered_map<entt::entity, entt::entity> *m_entity_map;
};

/**
 * @brief Reads data from the archive into the registry. Only assigns/replaces
 *      components for entities that exist in the target registry.
 * @param archive An input archive.
 * @param snapshot The snapshot reader which loads data into the registry.
 */
template<typename Archive, typename... Component>
void serialize(Archive &archive, registry_snapshot_reader<Component...> &snapshot) {
    static_assert(Archive::is_input::value, "Input archive expected.");

    size_t num_entities;
    archive(num_entities);

    for (size_t i = 0; i < num_entities; ++i) {
        entt::entity entity;
        archive(entity);

        size_t num_components;
        archive(num_components);

        for (size_t j = 0; j < num_components; ++j) {
            size_t comp_id;
            archive(comp_id);
            snapshot.visit(entity, comp_id, [&snapshot, &archive, &entity] (auto &&comp) {
                archive(comp);
                if (snapshot.m_registry->template valid(entity)) {
                    snapshot.m_registry->template assign_or_replace<std::decay_t<decltype(comp)>>(entity, comp);
                }
            });
        }
    }

    // Read "unmapped" entities which are encoded separately.
    /*size_t num_unmapped_entities;
    archive(num_unmapped_entities);

    for (size_t i = 0; i < num_unmapped_entities; ++i) {
        entt::entity remote_entity;
        archive(remote_entity);
        entt::entity local_entity;

        if (snapshot.m_entity_map.count(remote_entity)) {
            local_entity = snapshot.m_entity_map.at(remote_entity);
        } else {
            local_entity = snapshot.m_registry->template create();
            snapshot.m_entity_map[remote_entity] = local_entity;
        }

        size_t num_components;
        archive(num_components);

        for (size_t j = 0; j < num_components; ++j) {
            size_t comp_id;
            archive(comp_id);
            snapshot.visit(local_entity, comp_id, [&snapshot, &archive, &local_entity] (auto &&comp) {
                archive(comp);
                if (snapshot.m_registry->template valid(local_entity)) {
                    snapshot.m_registry->template assign_or_replace<std::decay_t<decltype(comp)>>(local_entity, comp);
                }
            });
        }
    }*/
}

//-----------------------------------------------------------------------------

/**
 * @brief Exports data from the registry mapping entities into the corresponding
 *      entities in another registry. The data can later be loaded into the
 *      other registry using `registry_snapshot_reader`. It only exports entities
 *      that are also present in the map.
 */
template<typename... Component>
class registry_snapshot_exporter: public registry_snapshot_writer<Component...> {
public:
    registry_snapshot_exporter(entt::registry &reg, entity_map &map) 
        : registry_snapshot_writer<Component...>(reg)
        , m_map(&map)
    {}

    template<typename Archive, typename... Ts>
    friend void serialize(Archive &, registry_snapshot_exporter<Ts...> &);

private:
    entity_map *m_map;
};

template<typename Archive, typename... Component>
void serialize(Archive &archive, registry_snapshot_exporter<Component...> &snapshot) {
    static_assert(Archive::is_output::value, "Output archive expected.");
    
    size_t num_entities = 0;
    for (auto entity : snapshot.m_entities) {
        if (snapshot.m_map->has_loc(entity)) {
            ++num_entities;
        }
    }
    archive(num_entities);

    for (auto entity : snapshot.m_entities) {
        if (!snapshot.m_map->has_loc(entity)) continue;

        archive(snapshot.m_map->locrem(entity));

        size_t count = 0;
        for (auto comp_id : snapshot.m_component_ids) {
            if (snapshot.has(entity, comp_id)) {
                ++count;
            }
        }
        archive(count);

        for (auto comp_id : snapshot.m_component_ids) {
            snapshot.try_visit(entity, comp_id, [&archive, &comp_id] (auto *comp) {
                if (comp) {
                    archive(comp_id);
                    archive(*comp);
                }
            });
        }
    }

    // Write entities not present in the entity map separately.
    /* auto num_unmapped_entities = snapshot.m_entities.size() - num_entities;
    archive(num_unmapped_entities);

    for (auto entity : snapshot.m_entities) {
        if (snapshot.m_map->has_loc(entity)) continue;

        archive(entity);

        size_t count = 0;
        for (auto comp_id : snapshot.m_component_ids) {
            if (snapshot.has(entity, comp_id)) {
                ++count;
            }
        }
        archive(count);

        for (auto comp_id : snapshot.m_component_ids) {
            snapshot.try_visit(entity, comp_id, [&archive, &comp_id] (auto *comp) {
                if (comp) {
                    archive(comp_id);
                    archive(*comp);
                }
            });
        }
    } */
}

//-----------------------------------------------------------------------------

/**
 * @brief Imports entities and components from one registry into another, 
 *      creating a mapping between entities. An existing map must be provided.
 *      If the external entity is not yet present in the map, a new entity will
 *      be created in the registry plus a mapping will be added connecting local
 *      to extenal entity. The same map should be reused in subsequent imports.
 */
template<typename... Component>
class registry_snapshot_importer: public registry_snapshot_reader<Component...> {
public:
    registry_snapshot_importer(entt::registry &reg, entity_map &map) 
        : registry_snapshot_reader<Component...>(reg)
        , m_map(&map)
    {}

    template<typename Archive, typename... Ts>
    friend void serialize(Archive &, registry_snapshot_importer<Ts...> &);

private:
    entity_map *m_map;
};

template<typename Archive, typename... Component>
void serialize(Archive &archive, registry_snapshot_importer<Component...> &snapshot) {
    static_assert(Archive::is_input::value, "Input archive expected.");

    size_t num_entities;
    archive(num_entities);

    for (size_t i = 0; i < num_entities; ++i) {
        entt::entity remote_entity;
        archive(remote_entity);
        
        entt::entity entity;
        if (!snapshot.m_map->has_rem(remote_entity)) {
            entity = snapshot.m_registry->template create();
            snapshot.m_map->insert(remote_entity, entity);
        } else {
            entity = snapshot.m_map->remloc(remote_entity);
        }

        size_t num_components;
        archive(num_components);

        for (size_t j = 0; j < num_components; ++j) {
            size_t comp_id;
            archive(comp_id);
            snapshot.visit(entity, comp_id, [&snapshot, &archive, &entity] (auto &&comp) {
                archive(comp);
                snapshot.m_registry->template assign_or_replace<std::decay_t<decltype(comp)>>(entity, comp);
            });
        }
    }
}

}

#endif // EDYN_SERIALIZATION_REGISTRY_S11N_HPP