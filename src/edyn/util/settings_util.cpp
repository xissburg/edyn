#include "edyn/util/settings_util.hpp"
#include "edyn/networking/context/client_network_context.hpp"
#include "edyn/simulation/stepper_async.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

void refresh_settings(entt::registry &registry)
{
    if (auto *stepper = registry.ctx().find<stepper_async>()) {
        stepper->settings_changed();
    }

    if (auto *ctx = registry.ctx().find<client_network_context>()) {
        auto &settings = registry.ctx().get<edyn::settings>();
        ctx->extrapolator->set_settings(settings);
    }
}

}
