#include "edyn/context/step_callback.hpp"
#include "edyn/context/settings.hpp"
#include "edyn/networking/context/client_network_context.hpp"
#include "edyn/simulation/stepper_async.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

namespace internal {
    void notify_settings(entt::registry &registry, const edyn::settings &settings) {
        if (auto *stepper = registry.ctx().find<stepper_async>()) {
            stepper->settings_changed();
        }

        if (auto *ctx = registry.ctx().find<client_network_context>()) {
            ctx->extrapolator->set_settings(settings);
        }
    }
}

void set_pre_step_callback(entt::registry &registry, step_callback_t func) {
    auto &settings = registry.ctx().get<edyn::settings>();
    settings.pre_step_callback = func;
    internal::notify_settings(registry, settings);
}

void set_post_step_callback(entt::registry &registry, step_callback_t func) {
    auto &settings = registry.ctx().get<edyn::settings>();
    settings.post_step_callback = func;
    internal::notify_settings(registry, settings);
}

void set_init_callback(entt::registry &registry, init_callback_t func) {
    auto &settings = registry.ctx().get<edyn::settings>();
    settings.init_callback = func;
    internal::notify_settings(registry, settings);
}

void set_deinit_callback(entt::registry &registry, init_callback_t func) {
    auto &settings = registry.ctx().get<edyn::settings>();
    settings.deinit_callback = func;
    internal::notify_settings(registry, settings);
}

}
