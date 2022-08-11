#include "edyn/edyn.hpp"
#include "edyn/collision/broadphase.hpp"
#include "edyn/collision/contact_event_emitter.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/collision/narrowphase.hpp"
#include "edyn/config/execution_mode.hpp"
#include "edyn/constraints/constraint.hpp"
#include "edyn/context/settings.hpp"
#include "edyn/networking/comp/entity_owner.hpp"
#include "edyn/simulation/stepper_async.hpp"
#include "edyn/simulation/stepper_sequential.hpp"
#include "edyn/sys/update_presentation.hpp"
#include "edyn/dynamics/material_mixing.hpp"
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

    entt::meta<entity_owner>().type()
        .data<&entity_owner::client_entity, entt::as_ref_t>("client_entity"_hs);
}

void attach(entt::registry &registry, const init_config &config) {
    init_meta();

    auto &dispatcher = job_dispatcher::global();
    auto num_workers = config.execution_mode == execution_mode::sequential ? 1 : config.num_worker_threads;

    if (!dispatcher.running()) {
        if (num_workers == 0) {
            dispatcher.start();
        } else {
            dispatcher.start(num_workers);
        }

        dispatcher.assure_current_queue();
    }

    auto &settings = registry.ctx().emplace<edyn::settings>();
    settings.execution_mode = config.execution_mode;

    registry.ctx().emplace<entity_graph>();
    registry.ctx().emplace<material_mix_table>();
    registry.ctx().emplace<contact_manifold_map>(registry);
    registry.ctx().emplace<contact_event_emitter>(registry);

    switch (config.execution_mode) {
    case execution_mode::sequential:
    case execution_mode::sequential_multithreaded:
        registry.ctx().emplace<broadphase>(registry);
        registry.ctx().emplace<narrowphase>(registry);
        registry.ctx().emplace<stepper_sequential>(registry, config.execution_mode == execution_mode::sequential_multithreaded);
        init_constraints(registry);
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
    registry.ctx().at<settings>().fixed_dt = dt;

    if (auto *stepper = registry.ctx().find<stepper_async>()) {
        stepper->settings_changed();
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
    // Run jobs scheduled in physics thread.
    job_dispatcher::global().once_current_queue();

    if (registry.ctx().contains<stepper_async>()) {
        registry.ctx().at<stepper_async>().update();
    } else if (registry.ctx().contains<stepper_sequential>()) {
        registry.ctx().at<stepper_sequential>().update();
    }

    if (is_paused(registry)) {
        snap_presentation(registry);
    } else {
        auto time = performance_time();
        update_presentation(registry, time);
    }

    // Clear actions after they've been pushed to island workers.
    auto &settings = registry.ctx().at<edyn::settings>();
    if (settings.clear_actions_func) {
        (*settings.clear_actions_func)(registry);
    }
}

void step_simulation(entt::registry &registry) {
    EDYN_ASSERT(is_paused(registry));

    if (auto *stepper = registry.ctx().find<stepper_async>()) {
        stepper->step_simulation();
    } else {
        registry.ctx().at<stepper_sequential>().step_simulation();
    }
}

void set_should_collide(entt::registry &registry, should_collide_func_t func) {
    registry.ctx().at<settings>().should_collide_func = func;

    if (auto *stepper = registry.ctx().find<stepper_async>()) {
        stepper->settings_changed();
    }
}

bool manifold_exists(entt::registry &registry, entt::entity first, entt::entity second) {
    return manifold_exists(registry, entity_pair{first, second});
}

bool manifold_exists(entt::registry &registry, entity_pair entities) {
    auto &manifold_map = registry.ctx().at<contact_manifold_map>();
    return manifold_map.contains(entities);
}

entt::entity get_manifold_entity(const entt::registry &registry, entt::entity first, entt::entity second) {
    return get_manifold_entity(registry, entity_pair{first, second});
}

entt::entity get_manifold_entity(const entt::registry &registry, entity_pair entities) {
    auto &manifold_map = registry.ctx().at<contact_manifold_map>();
    return manifold_map.get(entities);
}

entt::sink<entt::sigh<void(entt::entity)>> on_contact_started(entt::registry &registry) {
    return registry.ctx().at<contact_event_emitter>().contact_started_sink();
}

entt::sink<entt::sigh<void(entt::entity)>> on_contact_ended(entt::registry &registry) {
    return registry.ctx().at<contact_event_emitter>().contact_ended_sink();
}

entt::sink<entt::sigh<void(entt::entity, contact_manifold::contact_id_type)>> on_contact_point_created(entt::registry &registry) {
    return registry.ctx().at<contact_event_emitter>().contact_point_created_sink();
}

entt::sink<entt::sigh<void(entt::entity, contact_manifold::contact_id_type)>> on_contact_point_destroyed(entt::registry &registry) {
    return registry.ctx().at<contact_event_emitter>().contact_point_destroyed_sink();
}

vector3 get_gravity(const entt::registry &registry) {
    return registry.ctx().at<settings>().gravity;
}

void set_gravity(entt::registry &registry, vector3 gravity) {
    registry.ctx().at<settings>().gravity = gravity;

    auto view = registry.view<edyn::gravity, procedural_tag, rigidbody_tag>();

    for (auto entity : view) {
        view.get<edyn::gravity>(entity) = gravity;
        refresh<edyn::gravity>(registry, entity);
    }
}

unsigned get_solver_velocity_iterations(const entt::registry &registry) {
    return registry.ctx().at<settings>().num_solver_velocity_iterations;
}

void set_solver_velocity_iterations(entt::registry &registry, unsigned iterations) {
    auto &settings = registry.ctx().at<edyn::settings>();
    settings.num_solver_velocity_iterations = iterations;

    if (auto *stepper = registry.ctx().find<stepper_async>()) {
        stepper->settings_changed();
    }
}

unsigned get_solver_position_iterations(const entt::registry &registry) {
    return registry.ctx().at<settings>().num_solver_position_iterations;
}

void set_solver_position_iterations(entt::registry &registry, unsigned iterations) {
    auto &settings = registry.ctx().at<edyn::settings>();
    settings.num_solver_position_iterations = iterations;

    if (auto *stepper = registry.ctx().find<stepper_async>()) {
        stepper->settings_changed();
    }
}

unsigned get_solver_restitution_iterations(const entt::registry &registry) {
    return registry.ctx().at<settings>().num_restitution_iterations;
}

void set_solver_restitution_iterations(entt::registry &registry, unsigned iterations) {
    auto &settings = registry.ctx().at<edyn::settings>();
    settings.num_restitution_iterations = iterations;

    if (auto *stepper = registry.ctx().find<stepper_async>()) {
        stepper->settings_changed();
    }
}

unsigned get_solver_individual_restitution_iterations(const entt::registry &registry) {
    return registry.ctx().at<settings>().num_individual_restitution_iterations;
}

void set_solver_individual_restitution_iterations(entt::registry &registry, unsigned iterations) {
    auto &settings = registry.ctx().at<edyn::settings>();
    settings.num_individual_restitution_iterations = iterations;

    if (auto *stepper = registry.ctx().find<stepper_async>()) {
        stepper->settings_changed();
    }
}

void insert_material_mixing(entt::registry &registry, material::id_type material_id0,
                            material::id_type material_id1, const material_base &material) {
    auto &material_table = registry.ctx().at<material_mix_table>();
    material_table.insert({material_id0, material_id1}, material);

    if (auto *stepper = registry.ctx().find<stepper_async>()) {
        stepper->material_table_changed();
    }
}

execution_mode get_execution_mode(const entt::registry &registry) {
    auto &settings = registry.ctx().at<edyn::settings>();
    return settings.execution_mode;
}

}
