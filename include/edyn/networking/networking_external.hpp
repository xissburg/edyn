#ifndef EDYN_NETWORKING_NETWORKING_EXTERNAL_HPP
#define EDYN_NETWORKING_NETWORKING_EXTERNAL_HPP

#include <tuple>
#include <entt/entity/registry.hpp>
#include "edyn/networking/comp/networked_comp.hpp"
#include "edyn/networking/comp/transient_comp.hpp"
#include "edyn/networking/context/client_networking_context.hpp"
#include "edyn/networking/context/server_networking_context.hpp"

namespace edyn {

template<typename... Component, typename... Transient, typename... NonProcedural>
void register_networked_components(entt::registry &registry,
                                   std::tuple<Transient...> transient_external,
                                   std::tuple<NonProcedural...> non_procedural_external) {
    auto external = std::tuple<Component...>{};
    auto all = std::tuple_cat(networked_components, external);
    auto transient_all = std::tuple_cat(transient_components, transient_external);
    auto non_procedural_all = non_procedural_external;

    if (auto *ctx = registry.try_ctx<client_networking_context>()) {
        ctx->pool_snapshot_importer.reset(new client_pool_snapshot_importer_impl(all, non_procedural_all));
        ctx->pool_snapshot_exporter.reset(new client_pool_snapshot_exporter_impl(all, transient_all, non_procedural_all));
    }

    if (auto *ctx = registry.try_ctx<server_networking_context>()) {
        ctx->pool_snapshot_importer.reset(new server_pool_snapshot_importer_impl(all, non_procedural_all));
        ctx->pool_snapshot_exporter.reset(new server_pool_snapshot_exporter_impl(all, transient_all));
    }

    g_pool_snapshot_serializer.ptr.reset(new pool_snapshot_serializer_impl(all));
}

inline void unregister_networked_components(entt::registry &registry) {
    if (auto *ctx = registry.try_ctx<client_networking_context>()) {
        ctx->pool_snapshot_importer.reset(new client_pool_snapshot_importer_impl(networked_components, {}));
        ctx->pool_snapshot_exporter.reset(new client_pool_snapshot_exporter_impl(networked_components, transient_components, {}));
    }

    if (auto *ctx = registry.try_ctx<server_networking_context>()) {
        ctx->pool_snapshot_importer.reset(new server_pool_snapshot_importer_impl(networked_components, {}));
        ctx->pool_snapshot_exporter.reset(new server_pool_snapshot_exporter_impl(networked_components, transient_components));
    }

    g_pool_snapshot_serializer.ptr.reset(new pool_snapshot_serializer_impl(networked_components));
}

}

#endif // EDYN_NETWORKING_NETWORKING_EXTERNAL_HPP
