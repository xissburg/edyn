#include "edyn/dynamics/world.hpp"
#include "edyn/sys/integrate_linvel.hpp"
#include "edyn/sys/integrate_linacc.hpp"
#include "edyn/sys/update_current_position.hpp"
#include "edyn/time/time.hpp"

namespace edyn {

world::world(entt::registry& reg) 
    : registry(&reg)
{

}

world::~world() {
    
}

void world::update(scalar dt) {
    // Current elapsed time plus residual from previous update.
    auto total_dt = residual_dt + dt;
    // Number of steps for this update.
    int n = std::floor(total_dt / fixed_dt);
    // Store remainder to be accumulated on the next update.
    residual_dt = total_dt - n * fixed_dt;

    for (int i = 0; i < n; ++i) {
        step(fixed_dt);
    }

    update_current_position(*registry, residual_dt - fixed_dt);

    update_signal.publish(dt);
}

void world::step(scalar dt) {
    integrate_linacc(*registry, dt);
    integrate_linvel(*registry, dt);
    ++step_;
}

void world::run() {
    running = true;

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
    running = false;
}

}