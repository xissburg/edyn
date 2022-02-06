#ifndef EDYN_NETWORKING_UTIL_CLIENT_POOL_SNAPSHOT_EXPORTER_HPP
#define EDYN_NETWORKING_UTIL_CLIENT_POOL_SNAPSHOT_EXPORTER_HPP

#include <entt/entity/registry.hpp>
#include "edyn/networking/packet/util/pool_snapshot.hpp"

namespace edyn {

class client_pool_snapshot_exporter {
public:
    virtual void export_all(const entt::registry &registry, entt::entity entity, std::vector<pool_snapshot> &pools) = 0;
    virtual void export_transient(const entt::registry &registry, entt::entity entity, std::vector<pool_snapshot> &pools) = 0;
    virtual void export_non_procedural(const entt::registry &registry, entt::entity entity, std::vector<pool_snapshot> &pools) = 0;
    virtual void export_by_type_id(const entt::registry &registry, entt::entity entity, entt::id_type id, std::vector<pool_snapshot> &pools) = 0;
};

template<typename... Components>
class client_pool_snapshot_exporter_impl : public client_pool_snapshot_exporter {
public:
    using insert_entity_components_func_t = void(const entt::registry &, entt::entity, std::vector<pool_snapshot> &);
    insert_entity_components_func_t *insert_transient_entity_components_func;
    insert_entity_components_func_t *insert_non_procedural_entity_components_func;

    template<typename... Transient, typename... NonProcedural>
    client_pool_snapshot_exporter_impl(std::tuple<Components...>, std::tuple<Transient...>, std::tuple<NonProcedural...>) {
        static_assert((!std::is_empty_v<NonProcedural> && ...));

        insert_transient_entity_components_func = [] (const entt::registry &registry, entt::entity entity,
                                                      std::vector<pool_snapshot> &pools) {
            const std::tuple<Components...> components;
            internal::pool_insert_select_entity_components<Transient...>(registry, entity, pools, components);
        };
        insert_non_procedural_entity_components_func = [] (const entt::registry &registry, entt::entity entity,
                                                           std::vector<pool_snapshot> &pools) {
            const std::tuple<Components...> components;
            internal::pool_insert_select_entity_components<NonProcedural...>(registry, entity, pools, components);
        };
    }

    void export_all(const entt::registry &registry, entt::entity entity, std::vector<pool_snapshot> &pools) override {
        const std::tuple<Components...> components;
        internal::pool_insert_entity_components_all(registry, entity, pools, components,
                                                  std::make_index_sequence<sizeof...(Components)>{});
    }

    void export_transient(const entt::registry &registry, entt::entity entity, std::vector<pool_snapshot> &pools) override {
        (*insert_transient_entity_components_func)(registry, entity, pools);
    }

    void export_non_procedural(const entt::registry &registry, entt::entity entity, std::vector<pool_snapshot> &pools) override {
        (*insert_non_procedural_entity_components_func)(registry, entity, pools);
    }

    void export_by_type_id(const entt::registry &registry, entt::entity entity, entt::id_type id, std::vector<pool_snapshot> &pools) override {
        const std::tuple<Components...> components;
        ((entt::type_id<Components>().seq() == id ?
            internal::pool_insert_select_entity_component<Components>(registry, entity, pools, components) : void(0)), ...);
    }
};

}

#endif // EDYN_NETWORKING_UTIL_CLIENT_POOL_SNAPSHOT_EXPORTER_HPP
