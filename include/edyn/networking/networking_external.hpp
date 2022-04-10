#ifndef EDYN_NETWORKING_NETWORKING_EXTERNAL_HPP
#define EDYN_NETWORKING_NETWORKING_EXTERNAL_HPP

#include <tuple>
#include <entt/core/fwd.hpp>
#include <entt/entity/registry.hpp>
#include "edyn/networking/comp/networked_comp.hpp"
#include "edyn/networking/context/client_network_context.hpp"
#include "edyn/networking/context/server_network_context.hpp"
#include "edyn/networking/extrapolation_input.hpp"
#include "edyn/util/registry_operation.hpp"

namespace edyn {

namespace internal {
    static auto make_default_is_networked_component_func() {
        return [](entt::id_type id) {
            bool result;
            std::apply([&](auto ... c) {
                result = ((id == entt::type_index<decltype(c)>::value()) || ...);
            }, networked_components);
            return result;
        };
    }

    static auto make_default_mark_replaced_network_dirty_func() {
        return [](entt::registry &registry,
                                       const registry_operation_collection &ops,
                                       const entity_map &emap,
                                       double timestamp) {
            ops.replace_for_each(networked_components, [&](entt::entity remote_entity, const auto &c) {
                auto local_entity = emap.at(remote_entity);

                if (registry.all_of<networked_tag>(local_entity)) {
                    auto &n_dirty = registry.get_or_emplace<network_dirty>(local_entity);
                    n_dirty.insert<std::decay_t<decltype(c)>>(timestamp);
                }
            });
        };
    }
}

extern bool(*g_is_networked_component)(entt::id_type);
extern bool(*g_is_networked_input_component)(entt::id_type);
extern void(*g_mark_replaced_network_dirty)(entt::registry &, const registry_operation_collection &,
                                            const entity_map &, double timestamp);

/**
 * @brief Register external networked components.
 * @tparam Component All external networked components.
 * @tparam Which of the components are input. These components are exclusively
 * controlled by the client that owns the entity that has such component.
 * @param registry Data source.
 * @param input Tuple of input components.
 */
template<typename... Component, typename... Input>
void register_networked_components(entt::registry &registry,
                                   std::tuple<Input...> input) {
    auto external = std::tuple<Component...>{};
    auto all = std::tuple_cat(networked_components, external);

    if (auto *ctx = registry.try_ctx<client_network_context>()) {
        ctx->snapshot_importer.reset(new client_snapshot_importer_impl(all));
        ctx->snapshot_exporter.reset(new client_snapshot_exporter_impl(all));
        ctx->is_input_component_func = [] (entt::id_type id) {
            return ((id == entt::type_index<Input>::value()) || ...);
        };
        ctx->state_history = std::make_shared<comp_state_history_impl<Input...>>();
    }

    if (auto *ctx = registry.try_ctx<server_network_context>()) {
        ctx->snapshot_importer.reset(new server_snapshot_importer_impl(all, input));
        ctx->snapshot_exporter.reset(new server_snapshot_exporter_impl(all));
    }

    g_make_pool_snapshot_data = create_make_pool_snapshot_data_function(all);

    g_is_networked_component = [](entt::id_type id) {
        return ((id == entt::type_index<Component>::value()) || ...);
    };

    g_is_networked_input_component = [](entt::id_type id) {
        return ((id == entt::type_index<Input>::value()) || ...);
    };

    g_mark_replaced_network_dirty = [](entt::registry &registry,
                                       const registry_operation_collection &ops,
                                       const entity_map &emap,
                                       double timestamp) {
        ops.replace_for_each<Component...>([&](entt::entity remote_entity, const auto &c) {
            auto local_entity = emap.at(remote_entity);

            if (registry.all_of<networked_tag>(local_entity)) {
                auto &n_dirty = registry.get_or_emplace<network_dirty>(local_entity);
                n_dirty.insert<std::decay_t<decltype(c)>>(timestamp);
            }
        });
    };
}

/**
 * @brief Removes previously registered external networked components.
 * @param registry Data source.
 */
inline void unregister_networked_components(entt::registry &registry) {
    if (auto *ctx = registry.try_ctx<client_network_context>()) {
        ctx->snapshot_importer.reset(new client_snapshot_importer_impl(networked_components));
        ctx->snapshot_exporter.reset(new client_snapshot_exporter_impl(networked_components));
        ctx->is_input_component_func = [] (entt::id_type) { return false; };
        ctx->state_history = std::make_shared<comp_state_history>();
    }

    if (auto *ctx = registry.try_ctx<server_network_context>()) {
        ctx->snapshot_importer.reset(new server_snapshot_importer_impl(networked_components, {}));
        ctx->snapshot_exporter.reset(new server_snapshot_exporter_impl(networked_components));
    }

    g_make_pool_snapshot_data = create_make_pool_snapshot_data_function(networked_components);
    g_is_networked_component = internal::make_default_is_networked_component_func();
    g_is_networked_input_component = [](entt::id_type id) { return false; };
    g_mark_replaced_network_dirty = internal::make_default_mark_replaced_network_dirty_func();
}

}

#endif // EDYN_NETWORKING_NETWORKING_EXTERNAL_HPP
