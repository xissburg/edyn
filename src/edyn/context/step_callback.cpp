#include "edyn/context/step_callback.hpp"
#include "edyn/context/settings.hpp"
#include "edyn/simulation/stepper_async.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

void set_pre_step_callback(entt::registry &registry, step_callback_t func) {
    registry.ctx().at<settings>().pre_step_callback = func;

    if (auto *stepper = registry.ctx().find<stepper_async>()) {
        stepper->settings_changed();
    }
}

void set_post_step_callback(entt::registry &registry, step_callback_t func) {
    registry.ctx().at<settings>().post_step_callback = func;

    if (auto *stepper = registry.ctx().find<stepper_async>()) {
        stepper->settings_changed();
    }
}

void remove_pre_step_callback(entt::registry &registry) {
    auto &settings = registry.ctx().at<edyn::settings>();
    settings.pre_step_callback = nullptr;

    if (auto *stepper = registry.ctx().find<stepper_async>()) {
        stepper->settings_changed();
    }
}

void remove_post_step_callback(entt::registry &registry) {
    auto &settings = registry.ctx().at<edyn::settings>();
    settings.post_step_callback = nullptr;

    if (auto *stepper = registry.ctx().find<stepper_async>()) {
        stepper->settings_changed();
    }
}

}
