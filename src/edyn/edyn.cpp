#include "edyn/edyn.hpp"
#include "edyn/context/settings.hpp"
#include "edyn/collision/broadphase_main.hpp"
#include "edyn/parallel/island_coordinator.hpp"
#include "edyn/sys/update_presentation.hpp"
#include "edyn/dynamics/material_mixing.hpp"

namespace edyn {

void init() {
    auto &dispatcher = job_dispatcher::global();
    if (!dispatcher.running()) {
        dispatcher.start();
        dispatcher.assure_current_queue();
    }
}

void deinit() {
    job_dispatcher::global().stop();
}

void attach(entt::registry &registry) {
    registry.ctx().emplace<settings>();
    registry.ctx().emplace<entity_graph>();
    registry.ctx().emplace<contact_manifold_map>(registry);
    registry.ctx().emplace<island_coordinator>(registry);
    registry.ctx().emplace<broadphase_main>(registry);
    registry.ctx().emplace<material_mix_table>();
}

void detach(entt::registry &registry) {
    registry.ctx().erase<settings>();
    registry.ctx().erase<entity_graph>();
    registry.ctx().erase<contact_manifold_map>();
    registry.ctx().erase<island_coordinator>();
    registry.ctx().erase<broadphase_main>();
    registry.ctx().erase<material_mix_table>();
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

    // Do island management. Merge updated entity state into main registry.
    registry.ctx().at<island_coordinator>().update();

    // Perform broad-phase between different islands and create contact manifolds
    // between them which will later cause islands to be merged into one.
    registry.ctx().at<broadphase_main>().update();

    if (is_paused(registry)) {
        snap_presentation(registry);
    } else {
        auto time = performance_time();
        update_presentation(registry, time);
    }
}

void step_simulation(entt::registry &registry) {
    EDYN_ASSERT(is_paused(registry));
    registry.ctx().at<island_coordinator>().step_simulation();
}

void remove_external_components(entt::registry &registry) {
    auto &settings = registry.ctx().at<edyn::settings>();
    settings.make_island_delta_builder = &make_island_delta_builder_default;
    settings.index_source.reset(new component_index_source_impl(shared_components));
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

void insert_material_mixing(entt::registry &registry, unsigned material_id0,
                            unsigned material_id1, const material_base &material) {
    auto &material_table = registry.ctx().at<material_mix_table>();
    material_table.insert({material_id0, material_id1}, material);
    registry.ctx().at<island_coordinator>().material_table_changed();
}

}
