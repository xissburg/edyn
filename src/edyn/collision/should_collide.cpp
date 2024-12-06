#include "edyn/collision/should_collide.hpp"
#include "edyn/comp/collision_filter.hpp"
#include "edyn/comp/collision_exclusion.hpp"
#include "edyn/context/settings.hpp"
#include "edyn/networking/context/client_network_context.hpp"
#include "edyn/simulation/stepper_async.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

static bool should_exclude(const entt::registry &registry, entt::entity first, entt::entity second) {
    if (auto *exclusion = registry.try_get<collision_exclusion>(first)) {
        for (unsigned i = 0; i < exclusion->num_entities(); ++i) {
            if (exclusion->entity[i] == second) {
                return true;
            }
        }
    }

    return false;
}

bool should_collide_default(const entt::registry &registry, entt::entity first, entt::entity second) {
    if (first == second) {
        return false;
    }

    auto filter_view = registry.view<collision_filter>();
    auto first_has_filter = filter_view.contains(first);
    auto second_has_filter = filter_view.contains(second);

    if (first_has_filter && second_has_filter) {
        auto [filter0] = filter_view.get(first);
        auto [filter1] = filter_view.get(second);

        if ((filter0.group & filter1.mask) == 0 ||
            (filter1.group & filter0.mask) == 0) {
            return false;
        }
    } else if (first_has_filter || second_has_filter) {
        // If only one of them has a filter, use the default group and mask
        // values for the other.
        auto filter = first_has_filter ? std::get<0>(filter_view.get(first)) : std::get<0>(filter_view.get(second));

        if ((filter.group & collision_filter::all_groups) == 0 ||
            (filter.mask & collision_filter::all_groups) == 0) {
            return false;
        }
    }

    if (should_exclude(registry, first, second) ||
        should_exclude(registry, second, first)) {
        return false;
    }

    return true;
}

void set_should_collide(entt::registry &registry, should_collide_func_t func) {
    auto &settings = registry.ctx().get<edyn::settings>();
    settings.should_collide_func = func;

    if (auto *stepper = registry.ctx().find<stepper_async>()) {
        stepper->settings_changed();
    }

    if (auto *ctx = registry.ctx().find<client_network_context>()) {
        ctx->extrapolator->set_settings(settings);
    }
}

}
