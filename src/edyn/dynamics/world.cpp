#include "edyn/dynamics/world.hpp"
#include "edyn/sys/integrate_linvel.hpp"
#include "edyn/sys/integrate_linacc.hpp"
#include "edyn/sys/update_current_position.hpp"
#include "edyn/time/time.hpp"

namespace edyn {

world::world(entt::registry& reg) 
    : registry(&reg)
{
    connections.push_back(reg.on_construct<position>().connect<&world::on_construct_position>(*this));
    connections.push_back(reg.on_construct<linvel  >().connect<&world::on_construct_linvel>(*this));
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

    {
        std::lock_guard<std::mutex> lock(mutex);
        present();
        call_delegates();
    }

    update_signal.publish(dt);
}

void world::step(scalar dt) {
    integrate_linacc_priv(*registry, dt);
    integrate_linvel_priv(*registry, dt);
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

void world::update_current_positions() {
    std::lock_guard<std::mutex> lock(mutex);
    auto t = (double)performance_counter() / performance_frequency();
    update_current_position(*registry, t - local_time_ - fixed_dt);
}

void world::present() {
    auto view_pos = registry->view<const position_priv, position>();
    view_pos.each([] (auto, const position_priv &p0, position &p) {
        p = p0;
    });

    auto view_vel = registry->view<const linvel_priv, linvel>();
    view_vel.each([] (auto, const linvel_priv &v0, linvel &v) {
        v = v0;
    });
}

void world::call_delegates() {
    for (auto& d : delegates) {
        d(*registry);
    }
    delegates.clear();
}

void world::on_construct_position(entt::entity ent, entt::registry &registry, position& pos) {
    registry.assign<position_priv>(ent, pos);
}

void world::on_construct_linvel(entt::entity ent, entt::registry &registry, linvel& vel) {
    registry.assign<linvel_priv>(ent, vel);
}

}