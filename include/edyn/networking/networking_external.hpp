#ifndef EDYN_NETWORKING_NETWORKING_EXTERNAL_HPP
#define EDYN_NETWORKING_NETWORKING_EXTERNAL_HPP

#include <entt/entity/fwd.hpp>
#include <tuple>
#include <type_traits>
#include <entt/core/fwd.hpp>
#include <entt/entity/registry.hpp>
#include "edyn/comp/action_list.hpp"
#include "edyn/networking/comp/network_input.hpp"
#include "edyn/networking/comp/networked_comp.hpp"
#include "edyn/networking/context/client_network_context.hpp"
#include "edyn/networking/context/server_network_context.hpp"
#include "edyn/networking/util/input_state_history.hpp"
#include "edyn/replication/registry_operation.hpp"

namespace edyn {

/**
 * @brief Register external networked components.
 * @tparam Components All external networked components.
 * @tparam Actions All action types. Note that **actions are not components**.
 * Instead, they're stored in `edyn::action_list<Action>` which is an actual
 * component that is assigned to an entity and contains a list of actions.
 * @param registry Data source.
 * @param input Tuple of input components.
 */
template<typename... Components, typename... Actions>
void register_networked_components(entt::registry &registry, std::tuple<Actions...> actions = {}) {
    auto external = std::tuple<Components...>{};
    auto all = std::tuple_cat(networked_components, external);

    if (auto *ctx = registry.ctx().find<client_network_context>()) {
        ctx->snapshot_importer.reset(new client_snapshot_importer_impl(all));
        ctx->snapshot_exporter.reset(new client_snapshot_exporter_impl(registry, all, actions));

        auto input = std::tuple_cat(std::conditional_t<std::is_base_of_v<network_input, Components>,
                                    std::tuple<Components>, std::tuple<>>{}...);
        auto input_history = new input_state_history(input);
        auto input_history_ptr =
            std::shared_ptr<std::remove_pointer_t<decltype(input_history)>>(input_history);
        ctx->input_history.reset(new input_state_history_writer_impl(input_history_ptr));

        ctx->make_extrapolation_modified_comp = [](entt::registry &registry) {
            auto external = std::tuple<Components...>{};
            auto all = std::tuple_cat(networked_components, external);
            return std::unique_ptr<extrapolation_modified_comp>(
                new extrapolation_modified_comp_impl(registry, all));
        };

        auto input_history_reader = new input_state_history_reader_impl(input_history_ptr, actions);
        auto input_history_reader_ptr =
            std::shared_ptr<std::remove_pointer_t<decltype(input_history_reader)>>(input_history_reader);

        ctx->extrapolator->set_context_settings(input_history_reader_ptr,
                                                ctx->make_extrapolation_modified_comp);
    }

    if (auto *ctx = registry.ctx().find<server_network_context>()) {
        ctx->snapshot_importer.reset(new server_snapshot_importer_impl(all, actions));
        ctx->snapshot_exporter.reset(new server_snapshot_exporter_impl(registry, all));
    }

    g_make_pool_snapshot_data = create_make_pool_snapshot_data_function(all);
}

/**
 * @brief Removes previously registered external networked components.
 * @param registry Data source.
 */
inline void unregister_networked_components(entt::registry &registry) {
    if (auto *ctx = registry.ctx().find<client_network_context>()) {
        ctx->snapshot_importer.reset(new client_snapshot_importer_impl(networked_components));
        ctx->snapshot_exporter.reset(new client_snapshot_exporter_impl(registry, networked_components, {}));
        ctx->input_history = {};
    }

    if (auto *ctx = registry.ctx().find<server_network_context>()) {
        ctx->snapshot_importer.reset(new server_snapshot_importer_impl(networked_components, {}));
        ctx->snapshot_exporter.reset(new server_snapshot_exporter_impl(registry, networked_components));
    }

    g_make_pool_snapshot_data = create_make_pool_snapshot_data_function(networked_components);
}

}

#endif // EDYN_NETWORKING_NETWORKING_EXTERNAL_HPP
