#ifndef EDYN_NETWORKING_UTIL_CLIENT_POOL_SNAPSHOT_EXPORTER_HPP
#define EDYN_NETWORKING_UTIL_CLIENT_POOL_SNAPSHOT_EXPORTER_HPP

#include <entt/entity/fwd.hpp>
#include "edyn/networking/util/registry_snapshot.hpp"

namespace edyn {

class client_pool_snapshot_exporter {
public:
    virtual ~client_pool_snapshot_exporter() = default;
    virtual void export_all(const entt::registry &registry, registry_snapshot &snap) = 0;
    virtual void export_transient(const entt::registry &registry, registry_snapshot &snap) = 0;
    virtual void export_input(const entt::registry &registry, registry_snapshot &snap) = 0;
    virtual void export_by_type_id(const entt::registry &registry,
                                   entt::entity entity, entt::id_type id,
                                   registry_snapshot &snap) = 0;
};

template<typename... Components>
class client_pool_snapshot_exporter_impl : public client_pool_snapshot_exporter {
public:
    using insert_entity_components_func_t = void(const entt::registry &, registry_snapshot &);
    insert_entity_components_func_t *insert_transient_entity_components_func;
    insert_entity_components_func_t *insert_input_entity_components_func;

    template<typename... Transient, typename... Input>
    client_pool_snapshot_exporter_impl(std::tuple<Components...>, std::tuple<Transient...>, std::tuple<Input...>) {
        static_assert((!std::is_empty_v<Input> && ...));

        insert_transient_entity_components_func = [] (const entt::registry &registry, registry_snapshot &snap) {
            const std::tuple<Components...> components;
            internal::snapshot_insert_select_entity_components<Transient...>(registry, snap, components);
        };
        insert_input_entity_components_func = [] (const entt::registry &registry, registry_snapshot &snap) {
            const std::tuple<Components...> components;
            internal::snapshot_insert_select_entity_components<Input...>(registry, snap, components);
        };
    }

    void export_all(const entt::registry &registry, registry_snapshot &snap) override {
        const std::tuple<Components...> components;
        internal::snapshot_insert_entity_components_all(registry, snap, components,
                                                        std::make_index_sequence<sizeof...(Components)>{});
    }

    void export_transient(const entt::registry &registry, registry_snapshot &snap) override {
        (*insert_transient_entity_components_func)(registry, snap);
    }

    void export_input(const entt::registry &registry, registry_snapshot &snap) override {
        (*insert_input_entity_components_func)(registry, snap);
    }

    void export_by_type_id(const entt::registry &registry,
                           entt::entity entity, entt::id_type id,
                           registry_snapshot &snap) override {
        size_t i = 0;
        ((entt::type_id<Components>().seq() == id ?
            internal::snapshot_insert_entity<Components>(registry, entity, snap, i++) : (++i, void(0))), ...);
    }
};

}

#endif // EDYN_NETWORKING_UTIL_CLIENT_POOL_SNAPSHOT_EXPORTER_HPP
