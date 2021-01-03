#include "edyn/dynamics/world.hpp"
#include "edyn/sys/update_presentation.hpp"
#include "edyn/parallel/job_dispatcher.hpp"
#include "edyn/time/time.hpp"
#include <entt/entt.hpp>

#include <iostream>

namespace edyn {

world::world(entt::registry &registry) 
    : m_registry(&registry)
    , m_solver(registry)
    , m_bphase(registry)
    , m_island_coordinator(registry)
{
    job_dispatcher::global().assure_current_queue();
}

world::~world() {

}

void world::update() {
    auto freq = performance_frequency();
    auto time0 = performance_counter();
    // Run jobs scheduled in physics thread.
    job_dispatcher::global().once_current_queue();

    m_island_coordinator.update();
    m_bphase.update();

    if (m_paused) {
        snap_presentation(*m_registry);
    } else {
        auto time = (double)performance_counter() / (double)performance_frequency();
        update_presentation(*m_registry, time);
    }

    auto time1 = performance_counter();
    auto dt = (double)(time1 - time0) / freq;
    std::cout << dt * 1000 << std::endl;
}

void world::set_paused(bool paused) {
    m_paused = paused;
    m_island_coordinator.set_paused(paused);
}

void world::step() {
    m_island_coordinator.step_simulation();
}

}