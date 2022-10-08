#include "edyn/context/step_callback.hpp"
#include "edyn/context/settings.hpp"
#include "edyn/networking/context/client_network_context.hpp"
#include "edyn/simulation/stepper_async.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

void set_pre_step_callback(entt::registry &registry, step_callback_t func) {
    auto &settings = registry.ctx().at<edyn::settings>();
    settings.pre_step_callback = func;

    if (auto *stepper = registry.ctx().find<stepper_async>()) {
        stepper->settings_changed();
    }

    if (auto *ctx = registry.ctx().find<client_network_context>()) {
        ctx->extrapolator->set_settings(settings);
    }
}

void set_post_step_callback(entt::registry &registry, step_callback_t func) {
    auto &settings = registry.ctx().at<edyn::settings>();
    settings.post_step_callback = func;

    if (auto *stepper = registry.ctx().find<stepper_async>()) {
        stepper->settings_changed();
    }

    if (auto *ctx = registry.ctx().find<client_network_context>()) {
        ctx->extrapolator->set_settings(settings);
    }
}

void remove_pre_step_callback(entt::registry &registry) {
    auto &settings = registry.ctx().at<edyn::settings>();
    settings.pre_step_callback = nullptr;

    if (auto *stepper = registry.ctx().find<stepper_async>()) {
        stepper->settings_changed();
    }

    if (auto *ctx = registry.ctx().find<client_network_context>()) {
        ctx->extrapolator->set_settings(settings);
    }
}

void remove_post_step_callback(entt::registry &registry) {
    auto &settings = registry.ctx().at<edyn::settings>();
    settings.post_step_callback = nullptr;

    if (auto *stepper = registry.ctx().find<stepper_async>()) {
        stepper->settings_changed();
    }

    if (auto *ctx = registry.ctx().find<client_network_context>()) {
        ctx->extrapolator->set_settings(settings);
    }
}

}
