#include "edyn/config/solver_iteration_config.hpp"
#include "edyn/context/settings.hpp"
#include "edyn/networking/context/client_network_context.hpp"
#include "edyn/simulation/stepper_async.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

unsigned get_solver_velocity_iterations(const entt::registry &registry) {
    return registry.ctx().get<settings>().num_solver_velocity_iterations;
}

void set_solver_velocity_iterations(entt::registry &registry, unsigned iterations) {
    auto &settings = registry.ctx().get<edyn::settings>();
    settings.num_solver_velocity_iterations = iterations;

    if (auto *stepper = registry.ctx().find<stepper_async>()) {
        stepper->settings_changed();
    }

    if (auto *ctx = registry.ctx().find<client_network_context>()) {
        ctx->extrapolator->set_settings(settings);
    }
}

unsigned get_solver_position_iterations(const entt::registry &registry) {
    return registry.ctx().get<settings>().num_solver_position_iterations;
}

void set_solver_position_iterations(entt::registry &registry, unsigned iterations) {
    auto &settings = registry.ctx().get<edyn::settings>();
    settings.num_solver_position_iterations = iterations;

    if (auto *stepper = registry.ctx().find<stepper_async>()) {
        stepper->settings_changed();
    }

    if (auto *ctx = registry.ctx().find<client_network_context>()) {
        ctx->extrapolator->set_settings(settings);
    }
}

unsigned get_solver_restitution_iterations(const entt::registry &registry) {
    return registry.ctx().get<settings>().num_restitution_iterations;
}

void set_solver_restitution_iterations(entt::registry &registry, unsigned iterations) {
    auto &settings = registry.ctx().get<edyn::settings>();
    settings.num_restitution_iterations = iterations;

    if (auto *stepper = registry.ctx().find<stepper_async>()) {
        stepper->settings_changed();
    }

    if (auto *ctx = registry.ctx().find<client_network_context>()) {
        ctx->extrapolator->set_settings(settings);
    }
}

unsigned get_solver_individual_restitution_iterations(const entt::registry &registry) {
    return registry.ctx().get<settings>().num_individual_restitution_iterations;
}

void set_solver_individual_restitution_iterations(entt::registry &registry, unsigned iterations) {
    auto &settings = registry.ctx().get<edyn::settings>();
    settings.num_individual_restitution_iterations = iterations;

    if (auto *stepper = registry.ctx().find<stepper_async>()) {
        stepper->settings_changed();
    }

    if (auto *ctx = registry.ctx().find<client_network_context>()) {
        ctx->extrapolator->set_settings(settings);
    }
}

}
