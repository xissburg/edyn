#ifndef EDYN_SERIALIZATION_REGISTRY_S11N_HPP
#define EDYN_SERIALIZATION_REGISTRY_S11N_HPP

#include <cstdint>
#include <numeric>
#include <unordered_map>
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

    template<typename Comp, typename Archive>
    void serialize_component(Archive &archive, entt::entity entity) {
        if constexpr(std::is_empty_v<Comp>) {
            if (m_registry->has<Comp>(entity)) {
                archive(entity);
                size_t comp_id = index_of_v<Comp, Component...>;
                archive(comp_id);
            }
        } else {
            if (auto comp = m_registry->try_get<Comp>(entity)) {
                archive(entity);
                size_t comp_id = index_of_v<Comp, Component...>;
                archive(comp_id);
                archive(*comp);
            }
        }
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
    template<typename... Comp, typename Archive, typename It>
    void serialize(Archive &archive, It first, It last) {
        static_assert(Archive::is_output::value, "Output archive expected.");
    
        size_t num_entities = std::distance(first, last);
        archive(num_entities);

        size_t num_components = 0;

        for (auto it = first; it != last; ++it) {
            auto entity = *it;
            archive(entity);
            num_components += (m_registry->has<Comp>(entity) + ...);
        }

        archive(num_components);

        for (auto it = first; it != last; ++it) {
            auto entity = *it;
            (serialize_component<Comp>(archive, entity), ...);
        }
    }

protected:
    entt::registry *m_registry;
};

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

    template<typename Archive>
    void serialize(Archive &archive) {
        static_assert(Archive::is_input::value, "Input archive expected.");

        size_t num_entities;
        archive(num_entities);

        for (size_t i = 0; i < num_entities; ++i) {
            entt::entity entity;
            archive(entity);
        }

        size_t num_components;
        archive(num_components);

        for (size_t i = 0; i < num_components; ++i) {
            entt::entity entity;
            archive(entity);
            size_t comp_id;
            archive(comp_id);
            visit(entity, comp_id, [&] (auto &&comp) {
                archive(comp);
                if (m_registry->valid(entity)) {
                    m_registry->assign_or_replace<std::decay_t<decltype(comp)>>(entity, std::move(comp));
                }
            });
        }

        // Read "unmapped" entities which are encoded separately.
        /*size_t num_unmapped_entities;
        archive(num_unmapped_entities);

        for (size_t i = 0; i < num_unmapped_entities; ++i) {
            entt::entity remote_entity;
            archive(remote_entity);
            entt::entity local_entity;

            if (m_entity_map.count(remote_entity)) {
                local_entity = m_entity_map.at(remote_entity);
            } else {
                local_entity = m_registry->create();
                m_entity_map[remote_entity] = local_entity;
            }

            size_t num_components;
            archive(num_components);

            for (size_t j = 0; j < num_components; ++j) {
                size_t comp_id;
                archive(comp_id);
                visit(local_entity, comp_id, [&] (auto &&comp) {
                    archive(comp);
                    if (m_registry->valid(local_entity)) {
                        m_registry->assign_or_replace<std::decay_t<decltype(comp)>>(local_entity, comp);
                    }
                });
            }
        }*/
    }

protected:
    entt::registry *m_registry;
    //std::unordered_map<entt::entity, entt::entity> *m_entity_map;
};

//-----------------------------------------------------------------------------

/**
 * @brief Exports data from the registry mapping entities into the corresponding
 *      entities in another registry. The data can later be loaded into the
 *      other registry using `registry_snapshot_reader`. It only exports entities
 *      that are also present in the map.
 */
template<typename... Component>
class registry_snapshot_exporter {
    
    template<typename Other, typename Type, typename Member>
    void update_child_entity(Other &instance, Member Type:: *member) const {
        if constexpr(!std::is_same_v<Other, Type>) {
            return;
        } else if constexpr(std::is_same_v<Member, entt::entity>) {
            instance.*member = m_map->locrem(instance.*member);
        } else {
            // Attempt to use member as a container of entities.
            for(auto &ent : instance.*member) {
                ent = m_map->locrem(ent);
            }
        }
    }

    template<typename Comp, typename Archive, typename... Type, typename... Member>
    void serialize_component(Archive &archive, entt::entity entity, Member Type:: *...member) const {

        if constexpr(std::is_empty_v<Comp>) {
            if (m_registry->has<Comp>(entity)) {
                archive(m_map->locrem(entity));
                auto comp_id = index_of_v<Comp, Component...>;
                archive(comp_id);
            }
        } else {
            if (auto comp = m_registry->try_get<Comp>(entity)) {
                archive(m_map->locrem(entity));
                auto comp_id = index_of_v<Comp, Component...>;
                archive(comp_id);
                
                auto remote_comp = *comp;
                (update_child_entity(remote_comp, member), ...);
                archive(remote_comp);
            }
        }
    }
    
public:
    using tuple_type = std::tuple<Component...>;

    registry_snapshot_exporter(entt::registry &reg, entity_map &map) 
        : m_registry(&reg)
        , m_map(&map)
    {}

    template<typename... Comp, typename Archive, typename It, typename... Type, typename... Member>
    void serialize(Archive &archive, It first, It last, Member Type:: *...member) {
        static_assert(Archive::is_output::value, "Output archive expected.");

        auto num_entities = std::accumulate(first, last, size_t{0}, [&] (size_t count, entt::entity e) { return count + m_map->has_loc(e); });
        archive(num_entities);

        size_t num_components = 0;

        for (auto it = first; it != last; ++it) {
            auto entity = *it;
            if (!m_map->has_loc(entity)) continue;

            archive(m_map->locrem(entity));
            num_components += (m_registry->has<Comp>(entity) + ...);
        }

        archive(num_components);

        for (auto it = first; it != last; ++it) {
            auto entity = *it;
            (serialize_component<Comp>(archive, entity, member...), ...);
        }

        // Write entities not present in the entity map separately.
        /* 
        auto total_entities = std::distance(first, last);
        auto num_unmapped_entities = total_entities - num_entities;
        archive(num_unmapped_entities);

        for (auto entity : m_entities) {
            if (snapshot.m_map->has_loc(entity)) continue;

            archive(entity);

            size_t count = 0;
            for (auto comp_id : m_component_ids) {
                if (has(entity, comp_id)) {
                    ++count;
                }
            }
            archive(count);

            for (auto comp_id : m_component_ids) {
                try_visit(entity, comp_id, [&archive, &comp_id] (auto *comp) {
                    if (comp) {
                        archive(comp_id);
                        archive(*comp);
                    }
                });
            }
        } */
    }

private:
    entt::registry *m_registry;
    entity_map *m_map;
};

//-----------------------------------------------------------------------------

/**
 * @brief Imports entities and components from one registry into another, 
 *      creating a mapping between entities. An existing map must be provided.
 *      If the external entity is not yet present in the map, a new entity will
 *      be created in the registry plus a mapping will be added connecting local
 *      to extenal entity. The same map should be reused in subsequent imports.
 */
template<typename... Component>
class registry_snapshot_importer {

    template<typename Func, size_t... Indexes>
    void visit(entt::entity entity, size_t comp_id, Func f, std::index_sequence<Indexes...>) {
        ((comp_id == Indexes ? f(std::tuple_element_t<Indexes, tuple_type>{}) : (void)0), ...);
    }

    template<typename Func>
    void visit(entt::entity entity, size_t comp_id, Func f) {
        visit(entity, comp_id, f, std::make_index_sequence<sizeof...(Component)>{});
    }

    template<typename Other, typename Type, typename Member>
    void update_child_entity(Other &instance, Member Type:: *member) const {
        if constexpr(!std::is_same_v<Other, Type>) {
            return;
        } else if constexpr(std::is_same_v<Member, entt::entity>) {
            instance.*member = m_map->remloc(instance.*member);
        } else {
            // Attempt to use member as a container of entities.
            for(auto &ent : instance.*member) {
                ent = m_map->remloc(ent);
            }
        }
    }
    

public:
    using tuple_type = std::tuple<Component...>;

    registry_snapshot_importer(entt::registry &reg, entity_map &map) 
        : m_registry(&reg)
        , m_map(&map)
    {}

    template<typename Archive, typename... Type, typename... Member>
    void serialize(Archive &archive, Member Type:: *...member) {
        static_assert(Archive::is_input::value, "Input archive expected.");

        size_t num_entities;
        archive(num_entities);

        for (size_t i = 0; i < num_entities; ++i) {
            entt::entity remote_entity;
            archive(remote_entity);

            entt::entity entity;
            if (m_map->has_rem(remote_entity)) {
                entity = m_map->remloc(remote_entity);
            } else {
                entity = m_registry->create();
                m_map->insert(remote_entity, entity);
            }
        }

        size_t num_components;
        archive(num_components);

        for (size_t i = 0; i < num_components; ++i) {
            entt::entity remote_entity;
            archive(remote_entity);
            auto entity = m_map->remloc(remote_entity);

            size_t comp_id;
            archive(comp_id);

            visit(entity, comp_id, [&] (auto &&comp) {
                archive(comp);
                (update_child_entity(comp, member), ...);

                if (m_registry->valid(entity)) {
                    m_registry->assign_or_replace<std::decay_t<decltype(comp)>>(entity, comp);
                }
            });
        }
    }

private:
    entt::registry *m_registry;
    entity_map *m_map;
};

}

#endif // EDYN_SERIALIZATION_REGISTRY_S11N_HPP