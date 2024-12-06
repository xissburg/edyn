#include "edyn/networking/networking.hpp"
#include "edyn/config/config.h"
#include "edyn/networking/sys/client_side.hpp"
#include "edyn/networking/sys/server_side.hpp"
#include "edyn/networking/context/client_network_context.hpp"
#include "edyn/networking/context/server_network_context.hpp"
#include "edyn/networking/comp/remote_client.hpp"
#include "edyn/simulation/stepper_async.hpp"
#include <entt/entity/registry.hpp>
#include <variant>

namespace edyn {

static auto & get_client_settings(entt::registry &registry) {
    auto &settings = registry.ctx().get<edyn::settings>();
    EDYN_ASSERT(std::holds_alternative<client_network_settings>(settings.network_settings));
    auto &client_settings = std::get<client_network_settings>(settings.network_settings);
    return client_settings;
}

template<typename Func>
void edit_client_settings(entt::registry &registry, Func func) {
    auto &client_settings = get_client_settings(registry);

    if (func(client_settings)) {
        if (auto *stepper = registry.ctx().find<stepper_async>()) {
            stepper->settings_changed();
        }

        if (auto *ctx = registry.ctx().find<client_network_context>()) {
            auto &settings = registry.ctx().get<edyn::settings>();
            ctx->extrapolator->set_settings(settings);
        }
    }
}

void set_network_client_snapshot_rate(entt::registry &registry, double rate) {
    EDYN_ASSERT(rate > 0);
    edit_client_settings(registry, [rate](auto &client_settings) {
        auto changed = client_settings.snapshot_rate != rate;
        client_settings.snapshot_rate = rate;
        return changed;
    });
}

double get_network_client_snapshot_rate(entt::registry &registry) {
    return get_client_settings(registry).snapshot_rate;
}

void set_network_client_round_trip_time(entt::registry &registry, double rtt) {
    EDYN_ASSERT(!(rtt < 0));
    edit_client_settings(registry, [rtt](auto &client_settings) {
        auto changed = client_settings.round_trip_time != rtt;
        client_settings.round_trip_time = rtt;
        return changed;
    });
}

double get_network_client_round_trip_time(entt::registry &registry) {
    return get_client_settings(registry).round_trip_time;
}

void set_network_client_extrapolation_enabled(entt::registry &registry, bool enabled) {
    edit_client_settings(registry, [enabled](auto &client_settings) {
        auto changed = client_settings.extrapolation_enabled != enabled;
        client_settings.extrapolation_enabled = enabled;
        return changed;
    });
}

bool toggle_network_client_extrapolation_enabled(entt::registry &registry) {
    bool enabled;

    edit_client_settings(registry, [&enabled](auto &client_settings) {
        enabled = client_settings.extrapolation_enabled;
        enabled = !enabled;
        client_settings.extrapolation_enabled = enabled;
        return true;
    });

    return enabled;
}

bool get_network_client_extrapolation_enabled(entt::registry &registry) {
    return get_client_settings(registry).extrapolation_enabled;
}

void set_network_client_discontinuity_decay_rate(entt::registry &registry, scalar rate) {
    EDYN_ASSERT(rate > 0);
    edit_client_settings(registry, [rate](auto &client_settings) {
        auto changed = client_settings.discontinuity_decay_rate != rate;
        client_settings.discontinuity_decay_rate = rate;
        return changed;
    });
}

scalar get_network_client_discontinuity_decay_rate(entt::registry &registry) {
    return get_client_settings(registry).discontinuity_decay_rate;
}

entt::sink<entt::sigh<void(const packet::edyn_packet &)>>
network_client_packet_sink(entt::registry &registry) {
    auto &ctx = registry.ctx().get<client_network_context>();
    return ctx.packet_sink();
}

entt::sink<entt::sigh<void(entt::entity)>>
network_client_assigned_sink(entt::registry &registry) {
    auto &ctx = registry.ctx().get<client_network_context>();
    return ctx.client_assigned_sink();
}

entt::sink<entt::sigh<void(void)>>
network_client_extrapolation_timeout_sink(entt::registry &registry) {
    auto &ctx = registry.ctx().get<client_network_context>();
    return ctx.extrapolation_timeout_sink();
}

entt::sink<entt::sigh<void(entt::entity)>>
network_client_entity_entered_sink(entt::registry &registry) {
    auto &ctx = registry.ctx().get<client_network_context>();
    return ctx.entity_entered_sink();
}

entt::sink<entt::sigh<void(entt::entity)>>
network_client_instantiate_asset_sink(entt::registry &registry) {
    auto &ctx = registry.ctx().get<client_network_context>();
    return ctx.instantiate_asset_sink();
}

entt::sink<entt::sigh<void(entt::entity, const packet::edyn_packet &)>>
network_server_packet_sink(entt::registry &registry) {
    auto &ctx = registry.ctx().get<server_network_context>();
    return ctx.packet_sink();
}

}
