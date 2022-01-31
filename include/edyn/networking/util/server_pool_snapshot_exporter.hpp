#ifndef EDYN_NETWORKING_UTIL_SERVER_POOL_SNAPSHOT_EXPORTER_HPP
#define EDYN_NETWORKING_UTIL_SERVER_POOL_SNAPSHOT_EXPORTER_HPP

#include <entt/entity/registry.hpp>
#include "edyn/networking/packet/util/pool_snapshot.hpp"

namespace edyn {

class server_pool_snapshot_exporter {
public:
    virtual void export_all(entt::registry &registry, entt::entity entity, std::vector<pool_snapshot> &pools) = 0;
    virtual void export_transient(entt::registry &registry, entt::entity entity, std::vector<pool_snapshot> &pools) = 0;
};

template<typename... Components>
class server_pool_snapshot_exporter_impl : public server_pool_snapshot_exporter {
public:
    using insert_entity_components_func_t = void(entt::registry &, entt::entity, std::vector<pool_snapshot> &);
    insert_entity_components_func_t *insert_transient_entity_components_func;

    template<typename... Transient>
    server_pool_snapshot_exporter_impl(std::tuple<Components...>, std::tuple<Transient...>) {
        insert_transient_entity_components_func = [] (entt::registry &registry, entt::entity entity,
                                                 std::vector<pool_snapshot> &pools) {
            const std::tuple<Components...> components;
            internal::pool_insert_select_entity_components<Transient...>(registry, entity, pools, components);
        };
    }

    void export_all(entt::registry &registry, entt::entity entity, std::vector<pool_snapshot> &pools) override {
        const std::tuple<Components...> components;
        internal::pool_insert_entity_components_all(registry, entity, pools, components,
                                                    std::make_index_sequence<sizeof...(Components)>{});
    }

    void export_transient(entt::registry &registry, entt::entity entity, std::vector<pool_snapshot> &pools) override {
        (*insert_transient_entity_components_func)(registry, entity, pools);
    }
};

}

#endif // EDYN_NETWORKING_UTIL_SERVER_POOL_SNAPSHOT_EXPORTER_HPP
