#include "edyn/networking/networking.hpp"
#include "edyn/config/config.h"
#include "edyn/simulation/stepper_async.hpp"
#include <entt/entity/registry.hpp>
#include <variant>

namespace edyn {

static auto & get_client_settings(entt::registry &registry) {
    auto &settings = registry.ctx().at<edyn::settings>();
    EDYN_ASSERT(std::holds_alternative<client_network_settings>(settings.network_settings));
    auto &client_settings = std::get<client_network_settings>(settings.network_settings);
    return client_settings;
}

template<typename Func>
void edit_client_settings(entt::registry &registry, Func func) {
    auto &client_settings = get_client_settings(registry);
    func(client_settings);

    if (auto *stepper = registry.ctx().find<stepper_async>()) {
        stepper->settings_changed();
    }
}

void set_network_client_snapshot_rate(entt::registry &registry, double rate) {
    EDYN_ASSERT(rate > 0);
    edit_client_settings(registry, [rate](auto &client_settings) {
        client_settings.snapshot_rate = rate;
    });
}

double get_network_client_snapshot_rate(entt::registry &registry) {
    return get_client_settings(registry).snapshot_rate;
}

void set_network_client_round_trip_time(entt::registry &registry, double rtt) {
    EDYN_ASSERT(!(rtt < 0));
    edit_client_settings(registry, [rtt](auto &client_settings) {
        client_settings.round_trip_time = rtt;
    });
}

double get_network_client_round_trip_time(entt::registry &registry) {
    return get_client_settings(registry).round_trip_time;
}

void set_network_client_extrapolation_enabled(entt::registry &registry, bool enabled) {
    edit_client_settings(registry, [enabled](auto &client_settings) {
        client_settings.extrapolation_enabled = enabled;
    });
}

bool toggle_network_client_extrapolation_enabled(entt::registry &registry) {
    bool enabled;

    edit_client_settings(registry, [&enabled](auto &client_settings) {
        enabled = client_settings.extrapolation_enabled;
        enabled = !enabled;
        client_settings.extrapolation_enabled = enabled;
    });

    return enabled;
}

bool get_network_client_extrapolation_enabled(entt::registry &registry) {
    return get_client_settings(registry).extrapolation_enabled;
}

void set_network_client_discontinuity_decay_rate(entt::registry &registry, scalar rate) {
    EDYN_ASSERT(!(rate < 0) && rate < 1);
    edit_client_settings(registry, [rate](auto &client_settings) {
        client_settings.discontinuity_decay_rate = rate;
    });
}

scalar get_network_client_discontinuity_decay_rate(entt::registry &registry) {
    return get_client_settings(registry).discontinuity_decay_rate;
}

void set_network_client_action_time_threshold(entt::registry &registry, double threshold) {
    EDYN_ASSERT(!(threshold < 0));
    edit_client_settings(registry, [threshold](auto &client_settings) {
        client_settings.action_time_threshold = threshold;
    });
}

double get_network_client_action_time_threshold(entt::registry &registry) {
    return get_client_settings(registry).action_time_threshold;
}

entt::sink<entt::sigh<void(const packet::edyn_packet &)>> network_client_packet_sink(entt::registry &registry) {
    auto &ctx = registry.ctx().at<client_network_context>();
    return ctx.packet_sink();
}

entt::sink<entt::sigh<void(entt::entity)>> network_client_assigned_sink(entt::registry &registry) {
    auto &ctx = registry.ctx().at<client_network_context>();
    return ctx.client_assigned_sink();
}

entt::sink<entt::sigh<void(void)>> network_client_extrapolation_timeout_sink(entt::registry &registry) {
    auto &ctx = registry.ctx().at<client_network_context>();
    return ctx.extrapolation_timeout_sink();
}

entt::sink<entt::sigh<void(entt::entity, const packet::edyn_packet &)>> network_server_packet_sink(entt::registry &registry) {
    auto &ctx = registry.ctx().at<server_network_context>();
    return ctx.packet_sink();
}

}
