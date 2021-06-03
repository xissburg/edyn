#include "edyn/edyn.hpp"
#include "edyn/context/settings.hpp"
#include "edyn/parallel/island_coordinator.hpp"
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

    registry.ctx<island_coordinator>().update();
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

template<typename... Component>
void refresh(entt::registry &registry, entt::entity entity) {
    registry.ctx<island_coordinator>().refresh<Component...>(entity);
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
