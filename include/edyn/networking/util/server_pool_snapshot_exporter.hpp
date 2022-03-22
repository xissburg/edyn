#ifndef EDYN_NETWORKING_UTIL_SERVER_POOL_SNAPSHOT_EXPORTER_HPP
#define EDYN_NETWORKING_UTIL_SERVER_POOL_SNAPSHOT_EXPORTER_HPP

#include <entt/core/type_info.hpp>
#include <entt/entity/registry.hpp>
#include <type_traits>
#include "edyn/networking/comp/entity_owner.hpp"
#include "edyn/networking/util/pool_snapshot.hpp"
#include "edyn/comp/dirty.hpp"

namespace edyn {

class server_pool_snapshot_exporter {
public:
    virtual ~server_pool_snapshot_exporter() = default;
    virtual void export_all(const entt::registry &registry, registry_snapshot &snap) = 0;
    virtual void export_transient(const entt::registry &registry, registry_snapshot &snap,
                                  entt::entity dest_client_entity) = 0;
    virtual void export_by_type_id(const entt::registry &registry,
                                   entt::entity entity, entt::id_type id,
                                   registry_snapshot &snap) = 0;
    virtual void export_dirty_steady(const entt::registry &registry,
                                     entt::entity entity, const dirty &dirty,
                                     registry_snapshot &snap, entt::entity dest_client_entity) = 0;
    virtual bool contains_transient(const entt::registry &registry, entt::entity entity) const = 0;
};

template<typename... Components>
class server_pool_snapshot_exporter_impl : public server_pool_snapshot_exporter {
    template<typename Component, typename... Input>
    static void insert_transient_non_input(const entt::registry &registry,
                                           const std::vector<entt::entity> &entities,
                                           registry_snapshot &snap) {
        if constexpr(!std::disjunction_v<std::is_same<Component, Input>...>) {
            internal::pool_insert_entities<Component>(registry, entities.begin(), entities.end(),
                                                      snap, index_of_v<unsigned, Component, Components...>);
        }
    };

    using insert_entity_components_func_t = void(const entt::registry &, registry_snapshot &, entt::entity);
    insert_entity_components_func_t *m_insert_transient_entity_components_func;

    using should_export_steady_by_type_id_func_t = bool(const entt::registry &, entt::entity, entt::id_type, entt::entity);
    should_export_steady_by_type_id_func_t *m_should_export_steady_by_type_id;

    using contains_transient_func_t = bool(const entt::registry &, entt::entity);
    contains_transient_func_t *m_contains_transient;

public:
    template<typename... Transient, typename... Input>
    server_pool_snapshot_exporter_impl(std::tuple<Components...>, std::tuple<Transient...>, std::tuple<Input...>) {
        m_insert_transient_entity_components_func = [] (const entt::registry &registry,
                                                        registry_snapshot &snap,
                                                        entt::entity dest_client_entity) {
            // If the entity is owned by the destination client, only insert
            // transient components which are not input.
            std::vector<entt::entity> owned_entities, unowned_entities;

            for (auto entity : snap.entities) {
                if (auto *owner = registry.try_get<entity_owner>(entity); owner && owner->client_entity == dest_client_entity) {
                    owned_entities.push_back(entity);
                } else {
                    unowned_entities.push_back(entity);
                }
            }

            if (!owned_entities.empty()) {
                (insert_transient_non_input<Transient, Input...>(registry, owned_entities, snap), ...);
            }

            if (!unowned_entities.empty()) {
                (internal::pool_insert_entities<Transient>(registry, unowned_entities.begin(), unowned_entities.end(),
                                                           snap, index_of_v<unsigned, Transient, Components...>), ...);
            }
        };

        m_should_export_steady_by_type_id = [] (const entt::registry &registry, entt::entity entity,
                                              entt::id_type id, entt::entity dest_client_entity) {
            auto is_transient = ((entt::type_seq<Transient>::value() == id) || ...);

            if (auto *owner = registry.try_get<entity_owner>(entity); owner && owner->client_entity == dest_client_entity) {
                auto is_input = ((entt::type_seq<Input>::value() == id) || ...);
                return !is_input && !is_transient;
            } else {
                return !is_transient;
            }
        };

        m_contains_transient = [] (const entt::registry &registry, entt::entity entity) {
            return registry.any_of<Transient...>(entity);
        };
    }

    void export_all(const entt::registry &registry, registry_snapshot &snap) override {
        const std::tuple<Components...> components;
        internal::pool_insert_entity_components_all(registry, snap, components,
                                                    std::make_index_sequence<sizeof...(Components)>{});
    }

    void export_transient(const entt::registry &registry, registry_snapshot &snap, entt::entity dest_client_entity) override {
        (*m_insert_transient_entity_components_func)(registry, snap, dest_client_entity);
    }

    void export_by_type_id(const entt::registry &registry,
                           entt::entity entity, entt::id_type id,
                           registry_snapshot &snap) override {
        size_t i = 0;
        ((entt::type_id<Components>().seq() == id ?
            internal::pool_insert_entity<Components>(registry, entity, snap, i++) : (++i, void(0))), ...);
    }

    void export_dirty_steady(const entt::registry &registry,
                             entt::entity entity, const dirty &dirty,
                             registry_snapshot &snap, entt::entity dest_client_entity) override {
        for (auto id : dirty.updated_indexes) {
            if ((*m_should_export_steady_by_type_id)(registry, entity, id, dest_client_entity)) {
                export_by_type_id(registry, entity, id, snap);
            }
        }
    }

    bool contains_transient(const entt::registry &registry, entt::entity entity) const override {
        return (*m_contains_transient)(registry, entity);
    }
};

}

#endif // EDYN_NETWORKING_UTIL_SERVER_POOL_SNAPSHOT_EXPORTER_HPP
