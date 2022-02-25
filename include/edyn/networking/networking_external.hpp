#ifndef EDYN_NETWORKING_NETWORKING_EXTERNAL_HPP
#define EDYN_NETWORKING_NETWORKING_EXTERNAL_HPP

#include <tuple>
#include <entt/entity/registry.hpp>
#include "edyn/comp/shared_comp.hpp"
#include "edyn/networking/comp/networked_comp.hpp"
#include "edyn/networking/comp/transient_comp.hpp"
#include "edyn/networking/context/client_network_context.hpp"
#include "edyn/networking/context/server_network_context.hpp"
#include "edyn/networking/extrapolation_input.hpp"

namespace edyn {

template<typename... Component, typename... Transient, typename... Input>
void register_networked_components(entt::registry &registry,
                                   std::tuple<Transient...> transient_external,
                                   std::tuple<Input...> input) {
    auto external = std::tuple<Component...>{};
    auto all = std::tuple_cat(networked_components, external);
    auto transient_all = std::tuple_cat(transient_components, transient_external);
    auto shared_all = std::tuple_cat(shared_components, external);

    if (auto *ctx = registry.try_ctx<client_network_context>()) {
        ctx->pool_snapshot_importer.reset(new client_pool_snapshot_importer_impl(all, input));
        ctx->pool_snapshot_exporter.reset(new client_pool_snapshot_exporter_impl(all, transient_all, input));
        ctx->extrapolation_component_pool_import_func = internal::make_extrapolation_component_pools_import_func(shared_all);
        ctx->extrapolation_component_pool_import_by_id_func = internal::make_extrapolation_component_pools_import_by_id_func(shared_all);
        ctx->is_input_component_func = [] (entt::id_type id) {
            return ((id == entt::type_id<Input>().seq()) || ...);
        };
    }

    if (auto *ctx = registry.try_ctx<server_network_context>()) {
        ctx->pool_snapshot_importer.reset(new server_pool_snapshot_importer_impl(all, input));
        ctx->pool_snapshot_exporter.reset(new server_pool_snapshot_exporter_impl(all, transient_all, input));
    }

    g_make_pool_snapshot_data = create_make_pool_snapshot_data_function(all);
}

inline void unregister_networked_components(entt::registry &registry) {
    if (auto *ctx = registry.try_ctx<client_network_context>()) {
        ctx->pool_snapshot_importer.reset(new client_pool_snapshot_importer_impl(networked_components, {}));
        ctx->pool_snapshot_exporter.reset(new client_pool_snapshot_exporter_impl(networked_components, transient_components, {}));
        ctx->extrapolation_component_pool_import_func = internal::make_extrapolation_component_pools_import_func(shared_components);
        ctx->extrapolation_component_pool_import_by_id_func = internal::make_extrapolation_component_pools_import_by_id_func(shared_components);
        ctx->is_input_component_func = [] (entt::id_type) { return false; };
    }

    if (auto *ctx = registry.try_ctx<server_network_context>()) {
        ctx->pool_snapshot_importer.reset(new server_pool_snapshot_importer_impl(networked_components, {}));
        ctx->pool_snapshot_exporter.reset(new server_pool_snapshot_exporter_impl(networked_components, transient_components, {}));
    }

    g_make_pool_snapshot_data = create_make_pool_snapshot_data_function(networked_components);
}

}

#endif // EDYN_NETWORKING_NETWORKING_EXTERNAL_HPP
