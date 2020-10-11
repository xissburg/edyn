#include <type_traits>
#include "edyn/dynamics/world.hpp"
#include "edyn/sys/update_presentation.hpp"
#include "edyn/time/time.hpp"
#include "edyn/comp.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/parallel/job_dispatcher.hpp"
#include "edyn/serialization/s11n.hpp"
#include "edyn/dynamics/island_util.hpp"
#include "edyn/parallel/registry_snapshot.hpp"

namespace edyn {

world::world(entt::registry &reg) 
    : m_registry(&reg)
    , m_sol(reg)
    , m_bphase(reg)
    , m_island_coordinator(reg)
{
    m_connections.push_back(bphase.intersect_sink().connect<&world::on_broadphase_intersect>(*this));

    job_dispatcher::global().assure_current_queue();
}

world::~world() {
    
}

void world::on_broadphase_intersect(entt::entity e0, entt::entity e1) {
    merge_entities(e0, e1, entt::null);
}

void world::update(scalar dt) {
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

    m_update_signal.publish(dt);
}

void world::run() {
    m_running = true;

    const auto freq = performance_frequency();
    const auto timescale = 1.0 / freq;
    const auto t0 = performance_counter();
    auto ti = t0;

    // Use an Integral Controller to calculate the right amount of delay to
    // keep `dt` as close as possible to `fixed_dt`.
    const scalar I = 0.5;
    scalar delay_dt = 0;

    while (running) {
        const auto t = performance_counter();
        const auto dt = (t - ti) * timescale;
        update(dt);
        ti = t;
        local_time_ = t * timescale - residual_dt;

        auto err_dt = fixed_dt - dt;
        delay_dt += err_dt * I;

        delay(delay_dt * 1000);
    }
}

void world::quit() {
    m_running = false;
}

void world::set_paused(bool paused) {
    m_paused = paused;
    m_island_coordinator.set_paused(paused);
}

void world::step() {
    m_island_coordinator.step_simulation();
}

}