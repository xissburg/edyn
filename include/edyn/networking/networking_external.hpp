#ifndef EDYN_NETWORKING_NETWORKING_EXTERNAL_HPP
#define EDYN_NETWORKING_NETWORKING_EXTERNAL_HPP

#include <tuple>
#include <entt/entity/registry.hpp>
#include "edyn/networking/comp/networked_comp.hpp"
#include "edyn/networking/context/client_networking_context.hpp"
#include "edyn/networking/context/server_networking_context.hpp"
#include "edyn/networking/packet/util/pool_snapshot.hpp"
#include "edyn/networking/util/client_import_pool.hpp"
#include "edyn/networking/util/server_import_pool.hpp"

namespace edyn {

template<typename... Component, typename... TransientComponent>
void register_networked_components(entt::registry &registry,
                                   [[maybe_unused]] std::tuple<TransientComponent...>) {
    if (auto *ctx = registry.try_ctx<client_networking_context>()) {
        ctx->import_pool_func = [] (entt::registry &registry, const pool_snapshot &pool) {
            auto external = std::tuple<Component...>{};
            auto all = std::tuple_cat(networked_components, external);
            import_pool_client(registry, pool, all);
        };

        ctx->insert_entity_components_func = [] (entt::registry &registry, entt::entity entity,
                                                 std::vector<pool_snapshot> &pools) {
            auto external = std::tuple<Component...>{};
            auto all = std::tuple_cat(networked_components, external);
            insert_entity_components(registry, entity, pools, all,
                                     std::make_index_sequence<std::tuple_size_v<decltype(all)>>{});
        };

        ctx->insert_transient_components_func = [] (entt::registry &registry, entt::entity entity,
                                                    std::vector<pool_snapshot> &pools) {
            auto external = std::tuple<Component...>{};
            auto all = std::tuple_cat(networked_components, external);
            auto external_transient = std::tuple<TransientComponent...>{};
            auto all_transient = std::tuple_cat(transient_components, external_transient);
            insert_select_entity_components(registry, entity, pools, all, all_transient);
        };
    }

    if (auto *ctx = registry.try_ctx<server_networking_context>()) {
        ctx->import_pool_func = [] (entt::registry &registry, entt::entity client_entity, const pool_snapshot &pool) {
            auto external = std::tuple<Component...>{};
            auto all = std::tuple_cat(networked_components, external);
            import_pool_server(registry, client_entity, pool, all);
        };

        ctx->insert_entity_components_func = [] (entt::registry &registry, entt::entity entity,
                                                 std::vector<pool_snapshot> &pools) {
            auto external = std::tuple<Component...>{};
            auto all = std::tuple_cat(networked_components, external);
            insert_entity_components(registry, entity, pools, all,
                                     std::make_index_sequence<std::tuple_size_v<decltype(all)>>{});
        };

        ctx->insert_transient_components_func = [] (entt::registry &registry, entt::entity entity,
                                                    std::vector<pool_snapshot> &pools) {
            auto external = std::tuple<Component...>{};
            auto all = std::tuple_cat(networked_components, external);
            auto external_transient = std::tuple<TransientComponent...>{};
            auto all_transient = std::tuple_cat(transient_components, external_transient);
            insert_select_entity_components(registry, entity, pools, all, all_transient);
        };
    }

    auto external = std::tuple<Component...>{};
    auto all = std::tuple_cat(networked_components, external);
    g_pool_snapshot_serializer.ptr.reset(new pool_snapshot_serializer_impl(all));
}

template<typename... Component, typename... TransientComponent>
void register_networked_components(entt::registry &registry,
                                   [[maybe_unused]] std::tuple<Component...>,
                                   [[maybe_unused]] std::tuple<TransientComponent...> transient_components) {
    register_networked_components<Component...>(registry, transient_components);
}

inline void unregister_networked_components(entt::registry &registry) {
    if (auto *ctx = registry.try_ctx<client_networking_context>()) {
        ctx->import_pool_func = &import_pool_client_default;
        ctx->insert_entity_components_func = &insert_entity_components_default;
        ctx->insert_transient_components_func = &insert_transient_components_default;
    }

    if (auto *ctx = registry.try_ctx<server_networking_context>()) {
        ctx->import_pool_func = &import_pool_server_default;
        ctx->insert_entity_components_func = &insert_entity_components_default;
        ctx->insert_transient_components_func = &insert_transient_components_default;
    }

    g_pool_snapshot_serializer.ptr.reset(new pool_snapshot_serializer_impl(networked_components));
}

}

#endif // EDYN_NETWORKING_NETWORKING_EXTERNAL_HPP
