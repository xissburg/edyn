#include <type_traits>
#include "edyn/dynamics/world.hpp"
#include "edyn/sys/update_present_position.hpp"
#include "edyn/time/time.hpp"
#include "edyn/comp/constraint_row.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/delta_angvel.hpp"
#include "edyn/comp/delta_linvel.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/inertia.hpp"

namespace edyn {

void on_construct_constraint(entt::entity, entt::registry &registry, constraint &con) {
    std::visit([&] (auto&& value) {
        for (size_t i = 0; i < std::decay_t<decltype(value)>::num_rows; ++i) {
            auto e = registry.create();
            con.row[i] = e;
            auto &row = registry.assign<constraint_row>(e);
            row.entity = con.entity;
        }
    }, con.var);
}

void on_destroy_constraint(entt::entity entity, entt::registry &registry) {
    auto& con = registry.get<constraint>(entity);
    for (auto e : con.row) {
        if (e != entt::null) {
            registry.destroy(e);
        }
    }
}

void on_construct_mass(entt::entity entity, entt::registry &registry, mass &m) {
    EDYN_ASSERT(m > 0);
    registry.assign<mass_inv>(entity, m < EDYN_SCALAR_MAX ? 1 / m : 0);
}

void on_destroy_mass(entt::entity entity, entt::registry &registry) {
    if (registry.has<mass_inv>(entity)) {
        registry.remove<mass_inv>(entity);
    }
}

void on_construct_inertia(entt::entity entity, entt::registry &registry, inertia &i) {
    auto &invI = registry.assign<inertia_inv>(entity, i.x < EDYN_SCALAR_MAX ? 1 / i.x : 0, 
                                                      i.y < EDYN_SCALAR_MAX ? 1 / i.y : 0, 
                                                      i.z < EDYN_SCALAR_MAX ? 1 / i.z : 0);
    registry.assign<inertia_world_inv>(entity, diagonal(invI));
}

void on_destroy_inertia(entt::entity entity, entt::registry &registry) {
    if (registry.has<inertia_inv>(entity)) {
        registry.remove<inertia_inv>(entity);
    }

    if (registry.has<inertia_world_inv>(entity)) {
        registry.remove<inertia_world_inv>(entity);
    }
}

world::world(entt::registry &reg) 
    : registry(&reg)
    , sol(reg)
{
    connections.push_back(reg.on_construct<constraint>().connect<&on_construct_constraint>());
    connections.push_back(reg.on_destroy<constraint>().connect<&on_destroy_constraint>());

    connections.push_back(reg.on_construct<mass>().connect<&on_construct_mass>());
    connections.push_back(reg.on_destroy<mass>().connect<&on_destroy_mass>());

    connections.push_back(reg.on_construct<inertia>().connect<&on_construct_inertia>());
    connections.push_back(reg.on_destroy<inertia>().connect<&on_destroy_inertia>());
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

    update_present_position(*registry, residual_dt - fixed_dt);

    update_signal.publish(dt);
}

void world::step(scalar dt) {
    sol.update(dt);
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