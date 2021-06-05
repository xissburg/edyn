#include "edyn/edyn.hpp"
#include "edyn/context/settings.hpp"
#include "edyn/collision/broadphase_main.hpp"
#include "edyn/sys/update_presentation.hpp"

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
    registry.set<settings>();
    registry.set<entity_graph>();
    registry.set<contact_manifold_map>(registry);
    registry.set<island_coordinator>(registry);
    registry.set<broadphase_main>(registry);
}

void detach(entt::registry &registry) {
    registry.unset<settings>();
    registry.unset<entity_graph>();
    registry.unset<contact_manifold_map>();
    registry.unset<island_coordinator>();
    registry.unset<broadphase_main>();
}

scalar get_fixed_dt(entt::registry &registry) {
    return registry.ctx<settings>().fixed_dt;
}

void set_fixed_dt(entt::registry &registry, scalar dt) {
    registry.ctx<settings>().fixed_dt = dt;
    registry.ctx<island_coordinator>().set_fixed_dt(dt);
}

bool is_paused(entt::registry &registry) {
    return registry.ctx<settings>().paused;
}

void set_paused(entt::registry &registry, bool paused) {
    registry.ctx<settings>().paused = paused;
    registry.ctx<island_coordinator>().set_paused(paused);
}

void update(entt::registry &registry) {
    // Run jobs scheduled in physics thread.
    job_dispatcher::global().once_current_queue();

    // Do island management. Merge updated entity state into main registry.
    registry.ctx<island_coordinator>().update();

    // Perform broad-phase between different islands and create contact manifolds
    // between them which will later cause islands to be merged into one.
    registry.ctx<broadphase_main>().update();

    if (is_paused(registry)) {
        snap_presentation(registry);
    } else {
        auto time = (double)performance_counter() / (double)performance_frequency();
        update_presentation(registry, time);
    }
}

void step_simulation(entt::registry &registry) {
    EDYN_ASSERT(is_paused(registry));
    registry.ctx<island_coordinator>().step_simulation();
}

void remove_external_components(entt::registry &registry) {
    auto &settings = registry.ctx<edyn::settings>();
    settings.make_island_delta_builder = &make_island_delta_builder_default;
    registry.ctx<island_coordinator>().settings_changed();
}

void set_external_system_init(entt::registry &registry, external_system_func_t func) {
    registry.ctx<settings>().external_system_init = func;
    registry.ctx<island_coordinator>().settings_changed();
}

void set_external_system_pre_step(entt::registry &registry, external_system_func_t func) {
    registry.ctx<settings>().external_system_pre_step = func;
    registry.ctx<island_coordinator>().settings_changed();
}

void set_external_system_post_step(entt::registry &registry, external_system_func_t func) {
    registry.ctx<settings>().external_system_post_step = func;
    registry.ctx<island_coordinator>().settings_changed();
}

void set_external_system_functions(entt::registry &registry,
                                   external_system_func_t init_func,
                                   external_system_func_t pre_step_func,
                                   external_system_func_t post_step_func) {
    auto &settings = registry.ctx<edyn::settings>();
    settings.external_system_init = init_func;
    settings.external_system_pre_step = pre_step_func;
    settings.external_system_post_step = post_step_func;
    registry.ctx<island_coordinator>().settings_changed();
}

bool manifold_exists(entt::registry &registry, entity_pair entities) {
    auto &manifold_map = registry.ctx<contact_manifold_map>();
    return manifold_map.contains(entities);
}

entt::entity get_manifold_entity(entt::registry &registry, entity_pair entities) {
    auto &manifold_map = registry.ctx<contact_manifold_map>();
    return manifold_map.get(entities);
}

}
