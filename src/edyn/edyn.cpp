#include "edyn/edyn.hpp"
#include "edyn/collision/broadphase.hpp"
#include "edyn/collision/contact_event_emitter.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/collision/contact_manifold_map.hpp"
#include "edyn/collision/narrowphase.hpp"
#include "edyn/comp/child_list.hpp"
#include "edyn/comp/collision_exclusion.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/config/execution_mode.hpp"
#include "edyn/constraints/constraint.hpp"
#include "edyn/constraints/null_constraint.hpp"
#include "edyn/context/registry_operation_context.hpp"
#include "edyn/context/settings.hpp"
#include "edyn/networking/comp/entity_owner.hpp"
#include "edyn/networking/context/client_network_context.hpp"
#include "edyn/simulation/stepper_async.hpp"
#include "edyn/simulation/stepper_sequential.hpp"
#include "edyn/dynamics/material_mixing.hpp"
#include "edyn/time/time.hpp"
#include <entt/meta/factory.hpp>
#include <entt/core/hashed_string.hpp>

namespace edyn {

static void init_meta() {
    using namespace entt::literals;

    entt::meta<contact_manifold>().type()
        .data<&contact_manifold::body, entt::as_ref_t>("body"_hs);

    entt::meta<collision_exclusion>().type()
        .data<&collision_exclusion::entity, entt::as_ref_t>("entity"_hs);

    std::apply([&](auto ... c) {
        (entt::meta<decltype(c)>().type().template data<&decltype(c)::body, entt::as_ref_t>("body"_hs), ...);
    }, constraints_tuple);

    entt::meta<null_constraint>().type().data<&null_constraint::body, entt::as_ref_t>("body"_hs);

    entt::meta<entity_owner>().type()
        .data<&entity_owner::client_entity, entt::as_ref_t>("client_entity"_hs);

    entt::meta<island_resident>().type()
        .data<&island_resident::island_entity, entt::as_ref_t>("island_entity"_hs);

    entt::meta<parent_comp>().type()
        .data<&parent_comp::child, entt::as_ref_t>("child"_hs);

    entt::meta<child_list>().type()
        .data<&child_list::parent, entt::as_ref_t>("parent"_hs)
        .data<&child_list::next, entt::as_ref_t>("next"_hs);
}

void attach(entt::registry &registry, const init_config &config) {
    init_meta();

    auto &dispatcher = job_dispatcher::global();

    if (!dispatcher.running()) {
        auto num_workers = size_t{};

        switch (config.execution_mode) {
        case execution_mode::sequential:
            num_workers = 1; // One worker is needed for background tasks.
            break;
        case execution_mode::sequential_multithreaded:
            num_workers = config.num_worker_threads > 0 ?
                          config.num_worker_threads :
                          std::max(std::thread::hardware_concurrency(), 2u) - 1;
                          // Subtract one for the main thread.
            break;
        case execution_mode::asynchronous:
            num_workers = config.num_worker_threads > 0 ?
                          config.num_worker_threads :
                          std::max(std::thread::hardware_concurrency(), 3u) - 2;
                          // Subtract one for the main thread and another for the
                          // dedicated simulation worker thread.
            break;
        }

        dispatcher.start(num_workers);
        dispatcher.assure_current_queue();
    }

    auto &settings = registry.ctx().emplace<edyn::settings>();
    settings.execution_mode = config.execution_mode;

    registry.ctx().emplace<entity_graph>();
    registry.ctx().emplace<material_mix_table>();
    registry.ctx().emplace<contact_manifold_map>(registry);
    registry.ctx().emplace<contact_event_emitter>(registry);
    registry.ctx().emplace<registry_operation_context>();
    auto timestamp = config.timestamp ? *config.timestamp : performance_time();

    switch (config.execution_mode) {
    case execution_mode::sequential:
    case execution_mode::sequential_multithreaded:
        registry.ctx().emplace<broadphase>(registry);
        registry.ctx().emplace<narrowphase>(registry);
        registry.ctx().emplace<stepper_sequential>(registry,
                                                   config.execution_mode == execution_mode::sequential_multithreaded,
                                                   timestamp);
        break;
    case execution_mode::asynchronous:
        registry.ctx().emplace<stepper_async>(registry);
        break;
    }
}

void detach(entt::registry &registry) {
    registry.ctx().erase<settings>();
    registry.ctx().erase<entity_graph>();
    registry.ctx().erase<material_mix_table>();
    registry.ctx().erase<contact_manifold_map>();
    registry.ctx().erase<broadphase>();
    registry.ctx().erase<narrowphase>();
    registry.ctx().erase<stepper_async>();
    registry.ctx().erase<stepper_sequential>();

    job_dispatcher::global().stop();
}

scalar get_fixed_dt(const entt::registry &registry) {
    return registry.ctx().at<settings>().fixed_dt;
}

void set_fixed_dt(entt::registry &registry, scalar dt) {
    auto &settings = registry.ctx().at<edyn::settings>();
    settings.fixed_dt = dt;

    if (auto *stepper = registry.ctx().find<stepper_async>()) {
        stepper->settings_changed();
    }

    if (auto *ctx = registry.ctx().find<client_network_context>()) {
        ctx->extrapolator->set_settings(settings);
    }
}

void set_max_steps_per_update(entt::registry &registry, unsigned max_steps) {
    auto &settings = registry.ctx().at<edyn::settings>();
    settings.max_steps_per_update = max_steps;

    if (auto *stepper = registry.ctx().find<stepper_async>()) {
        stepper->settings_changed();
    }

    if (auto *ctx = registry.ctx().find<client_network_context>()) {
        ctx->extrapolator->set_settings(settings);
    }
}

bool is_paused(const entt::registry &registry) {
    return registry.ctx().at<settings>().paused;
}

void set_paused(entt::registry &registry, bool paused) {
    registry.ctx().at<settings>().paused = paused;

    if (auto *stepper = registry.ctx().find<stepper_async>()) {
        stepper->set_paused(paused);
    } else {
        registry.ctx().at<stepper_sequential>().set_paused(paused);
    }
}

void update(entt::registry &registry) {
    auto time = performance_time();
    update(registry, time);
}

void update(entt::registry &registry, double time) {
    // Run jobs scheduled to run in this thread.
    job_dispatcher::global().once_current_queue();

    if (registry.ctx().contains<stepper_async>()) {
        registry.ctx().at<stepper_async>().update(time);
    } else if (registry.ctx().contains<stepper_sequential>()) {
        registry.ctx().at<stepper_sequential>().update(time);
    }
}

void step_simulation(entt::registry &registry) {
    EDYN_ASSERT(is_paused(registry));

    if (auto *stepper = registry.ctx().find<stepper_async>()) {
        stepper->step_simulation();
    } else {
        auto time = performance_time();
        registry.ctx().at<stepper_sequential>().step_simulation(time);
    }
}

void step_simulation(entt::registry &registry, double time) {
    EDYN_ASSERT(is_paused(registry));

    if (auto *stepper = registry.ctx().find<stepper_async>()) {
        stepper->step_simulation();
    } else {
        registry.ctx().at<stepper_sequential>().step_simulation(time);
    }
}

execution_mode get_execution_mode(const entt::registry &registry) {
    auto &settings = registry.ctx().at<edyn::settings>();
    return settings.execution_mode;
}

}
