#include "edyn/edyn.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/collision/narrowphase.hpp"
#include "edyn/config/execution_mode.hpp"
#include "edyn/context/settings.hpp"
#include "edyn/networking/comp/entity_owner.hpp"
#include "edyn/simulation/island_coordinator.hpp"
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

    registry.ctx().emplace<settings>();
    registry.ctx().emplace<entity_graph>();
    registry.ctx().emplace<material_mix_table>();
    registry.ctx().emplace<contact_manifold_map>(registry);

    switch (config.execution_mode) {
    case execution_mode::sequential:
        registry.ctx().emplace<broadphase>(registry);
        registry.ctx().emplace<narrowphase>(registry);
        registry.ctx().emplace<stepper_sequential>(registry, false);
        break;
    case execution_mode::sequential_mt:
        registry.ctx().emplace<broadphase>(registry);
        registry.ctx().emplace<narrowphase>(registry);
        registry.ctx().emplace<stepper_sequential>(registry, true);
        break;
    case execution_mode::asynchronous:
        registry.ctx().emplace<island_coordinator>(registry);
        break;
    }
}

void detach(entt::registry &registry) {
    registry.ctx().erase<settings>();
    registry.ctx().erase<entity_graph>();
    registry.ctx().erase<material_mix_table>();
    registry.ctx().erase<contact_manifold_map>();
    registry.ctx().erase<island_coordinator>();
    registry.ctx().erase<stepper_sequential>();

    job_dispatcher::global().stop();

}

scalar get_fixed_dt(const entt::registry &registry) {
    return registry.ctx().at<settings>().fixed_dt;
}

void set_fixed_dt(entt::registry &registry, scalar dt) {
    registry.ctx().at<settings>().fixed_dt = dt;
    registry.ctx().at<island_coordinator>().settings_changed();
}

bool is_paused(const entt::registry &registry) {
    return registry.ctx().at<settings>().paused;
}

void set_paused(entt::registry &registry, bool paused) {
    registry.ctx().at<settings>().paused = paused;
    registry.ctx().at<island_coordinator>().set_paused(paused);
}

void update(entt::registry &registry) {
    // Run jobs scheduled in physics thread.
    job_dispatcher::global().once_current_queue();

    if (registry.ctx().contains<island_coordinator>()) {
        registry.ctx().at<island_coordinator>().update();
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
    registry.ctx().at<island_coordinator>().step_simulation();
}

void remove_external_components(entt::registry &registry) {
    auto &settings = registry.ctx().at<edyn::settings>();
    settings.make_reg_op_builder = &make_reg_op_builder_default;
    settings.index_source.reset(new component_index_source_impl(shared_components_t{}));
    settings.clear_actions_func = nullptr;
    registry.ctx().at<island_coordinator>().settings_changed();
}

void set_external_system_init(entt::registry &registry, external_system_func_t func) {
    registry.ctx().at<settings>().external_system_init = func;
    registry.ctx().at<island_coordinator>().settings_changed();
}

void set_external_system_pre_step(entt::registry &registry, external_system_func_t func) {
    registry.ctx().at<settings>().external_system_pre_step = func;
    registry.ctx().at<island_coordinator>().settings_changed();
}

void set_external_system_post_step(entt::registry &registry, external_system_func_t func) {
    registry.ctx().at<settings>().external_system_post_step = func;
    registry.ctx().at<island_coordinator>().settings_changed();
}

void set_external_system_functions(entt::registry &registry,
                                   external_system_func_t init_func,
                                   external_system_func_t pre_step_func,
                                   external_system_func_t post_step_func) {
    auto &settings = registry.ctx().at<edyn::settings>();
    settings.external_system_init = init_func;
    settings.external_system_pre_step = pre_step_func;
    settings.external_system_post_step = post_step_func;
    registry.ctx().at<island_coordinator>().settings_changed();
}

void remove_external_systems(entt::registry &registry) {
    auto &settings = registry.ctx().at<edyn::settings>();
    settings.external_system_init = nullptr;
    settings.external_system_pre_step = nullptr;
    settings.external_system_post_step = nullptr;
    registry.ctx().at<island_coordinator>().settings_changed();
}

void tag_external_entity(entt::registry &registry, entt::entity entity, bool procedural) {
    if (procedural) {
        registry.emplace<edyn::procedural_tag>(entity);
    }

    registry.emplace<edyn::external_tag>(entity);
    auto non_connecting = !procedural;
    auto node_index = registry.ctx().at<edyn::entity_graph>().insert_node(entity, non_connecting);
    registry.emplace<edyn::graph_node>(entity, node_index);
}

void set_should_collide(entt::registry &registry, should_collide_func_t func) {
    registry.ctx().at<settings>().should_collide_func = func;
    registry.ctx().at<island_coordinator>().settings_changed();
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
    return registry.ctx().at<island_coordinator>().contact_started_sink();
}

entt::sink<entt::sigh<void(entt::entity)>> on_contact_ended(entt::registry &registry) {
    return registry.ctx().at<island_coordinator>().contact_ended_sink();
}

entt::sink<entt::sigh<void(entt::entity, contact_manifold::contact_id_type)>> on_contact_point_created(entt::registry &registry) {
    return registry.ctx().at<island_coordinator>().contact_point_created_sink();
}

entt::sink<entt::sigh<void(entt::entity, contact_manifold::contact_id_type)>> on_contact_point_destroyed(entt::registry &registry) {
    return registry.ctx().at<island_coordinator>().contact_point_destroyed_sink();
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
    registry.ctx().at<island_coordinator>().settings_changed();
}

unsigned get_solver_position_iterations(const entt::registry &registry) {
    return registry.ctx().at<settings>().num_solver_position_iterations;
}

void set_solver_position_iterations(entt::registry &registry, unsigned iterations) {
    auto &settings = registry.ctx().at<edyn::settings>();
    settings.num_solver_position_iterations = iterations;
    registry.ctx().at<island_coordinator>().settings_changed();
}

unsigned get_solver_restitution_iterations(const entt::registry &registry) {
    return registry.ctx().at<settings>().num_restitution_iterations;
}

void set_solver_restitution_iterations(entt::registry &registry, unsigned iterations) {
    auto &settings = registry.ctx().at<edyn::settings>();
    settings.num_restitution_iterations = iterations;
    registry.ctx().at<island_coordinator>().settings_changed();
}

unsigned get_solver_individual_restitution_iterations(const entt::registry &registry) {
    return registry.ctx().at<settings>().num_individual_restitution_iterations;
}

void set_solver_individual_restitution_iterations(entt::registry &registry, unsigned iterations) {
    auto &settings = registry.ctx().at<edyn::settings>();
    settings.num_individual_restitution_iterations = iterations;
    registry.ctx().at<island_coordinator>().settings_changed();
}

void insert_material_mixing(entt::registry &registry, material::id_type material_id0,
                            material::id_type material_id1, const material_base &material) {
    auto &material_table = registry.ctx().at<material_mix_table>();
    material_table.insert({material_id0, material_id1}, material);
    registry.ctx().at<island_coordinator>().material_table_changed();
}

}
