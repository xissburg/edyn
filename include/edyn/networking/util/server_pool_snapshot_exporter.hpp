#ifndef EDYN_NETWORKING_UTIL_SERVER_POOL_SNAPSHOT_EXPORTER_HPP
#define EDYN_NETWORKING_UTIL_SERVER_POOL_SNAPSHOT_EXPORTER_HPP

#include <entt/core/type_info.hpp>
#include <entt/entity/registry.hpp>
#include <type_traits>
#include "edyn/networking/comp/entity_owner.hpp"
#include "edyn/networking/packet/util/pool_snapshot.hpp"
#include "edyn/comp/dirty.hpp"

namespace edyn {

class server_pool_snapshot_exporter {
public:
    virtual void export_all(const entt::registry &registry, entt::entity entity,
                            std::vector<pool_snapshot> &pools) = 0;
    virtual void export_transient(const entt::registry &registry, entt::entity entity,
                                  std::vector<pool_snapshot> &pools, entt::entity dest_client_entity) = 0;
    virtual void export_by_type_id(const entt::registry &registry, entt::entity entity,
                                   entt::id_type id, std::vector<pool_snapshot> &pools) = 0;
    virtual void export_dirty_steady(const entt::registry &registry, entt::entity entity, const dirty &dirty,
                                     std::vector<pool_snapshot> &pools, entt::entity dest_client_entity) = 0;
};

template<typename... Components>
class server_pool_snapshot_exporter_impl : public server_pool_snapshot_exporter {
    template<typename Component, typename... Input>
    static void insert_transient_non_input(const entt::registry &registry, entt::entity entity,
                                    std::vector<pool_snapshot> &pools) {
        if constexpr(!std::disjunction_v<std::is_same<Component, Input>...>) {
            const std::tuple<Components...> components;
            internal::pool_insert_select_entity_component<Component>(registry, entity, pools, components);
        }
    };

    using insert_entity_components_func_t = void(const entt::registry &, entt::entity, std::vector<pool_snapshot> &, entt::entity);
    insert_entity_components_func_t *insert_transient_entity_components_func;

    using should_export_transient_by_type_id_func_t = bool(const entt::registry &, entt::entity, entt::id_type, entt::entity);
    should_export_transient_by_type_id_func_t *should_export_transient_by_type_id;

public:
    template<typename... Transient, typename... Input>
    server_pool_snapshot_exporter_impl(std::tuple<Components...>, std::tuple<Transient...>, std::tuple<Input...>) {
        insert_transient_entity_components_func = [] (const entt::registry &registry, entt::entity entity,
                                                      std::vector<pool_snapshot> &pools, entt::entity dest_client_entity) {
            // If the entity is owned by the destination client, only insert
            // transient components which are not input.
            if (auto *owner = registry.try_get<entity_owner>(entity); owner && owner->client_entity == dest_client_entity) {
                (insert_transient_non_input<Transient, Input...>(registry, entity, pools), ...);
            } else {
                const std::tuple<Components...> components;
                internal::pool_insert_select_entity_components<Transient...>(registry, entity, pools, components);
            }
        };

        should_export_transient_by_type_id = [] (const entt::registry &registry, entt::entity entity,
                                                 entt::id_type id, entt::entity dest_client_entity) {
            auto is_transient = ((entt::type_seq<Transient>::value() == id) || ...);

            if (auto *owner = registry.try_get<entity_owner>(entity); owner && owner->client_entity == dest_client_entity) {
                auto is_input = ((entt::type_seq<Input>::value() == id) || ...);
                return !is_input && !is_transient;
            } else {
                return !is_transient;
            }
        };
    }

    void export_all(const entt::registry &registry, entt::entity entity,
                    std::vector<pool_snapshot> &pools) override {
        const std::tuple<Components...> components;
        internal::pool_insert_entity_components_all(registry, entity, pools, components,
                                                    std::make_index_sequence<sizeof...(Components)>{});
    }

    void export_transient(const entt::registry &registry, entt::entity entity,
                          std::vector<pool_snapshot> &pools, entt::entity dest_client_entity) override {
        (*insert_transient_entity_components_func)(registry, entity, pools, dest_client_entity);
    }

    void export_by_type_id(const entt::registry &registry, entt::entity entity,
                           entt::id_type id, std::vector<pool_snapshot> &pools) override {
        const std::tuple<Components...> components;
        ((entt::type_id<Components>().seq() == id ?
            internal::pool_insert_select_entity_component<Components>(registry, entity, pools, components) : void(0)), ...);
    }

    void export_dirty_steady(const entt::registry &registry, entt::entity entity, const dirty &dirty,
                             std::vector<pool_snapshot> &pools, entt::entity dest_client_entity) override {
        for (auto id : dirty.updated_indexes) {
            if ((*should_export_transient_by_type_id)(registry, entity, id, dest_client_entity)) {
                export_by_type_id(registry, entity, id, pools);
            }
        }
    }
};

}

#endif // EDYN_NETWORKING_UTIL_SERVER_POOL_SNAPSHOT_EXPORTER_HPP
