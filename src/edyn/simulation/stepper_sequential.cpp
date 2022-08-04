#include "edyn/simulation/stepper_sequential.hpp"
#include "edyn/context/settings.hpp"
#include "edyn/collision/broadphase.hpp"
#include "edyn/collision/contact_manifold_map.hpp"
#include "edyn/collision/narrowphase.hpp"
#include "edyn/core/entity_graph.hpp"
#include "edyn/dynamics/material_mixing.hpp"
#include "edyn/time/time.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

stepper_sequential::stepper_sequential(entt::registry &registry)
    : m_registry(&registry)
    , m_island_manager(registry)
    , m_raycast_service(registry)
    , m_poly_initializer(registry)
    , m_solver(registry)
{
    m_last_time = performance_time();
}

void stepper_sequential::step_simulation(double dt) {
    m_poly_initializer.init_new_shapes();
    m_registry->ctx().at<broadphase>().update();
    m_island_manager.update();
    m_registry->ctx().at<narrowphase>().update();
    m_solver.update(dt);
}

void stepper_sequential::update() {
    auto time = performance_time();
    auto elapsed = time - m_last_time;

    auto fixed_dt = m_registry->ctx().at<settings>().fixed_dt;
    m_accumulated_time += elapsed;
    auto num_steps = static_cast<int>(std::floor(m_accumulated_time / fixed_dt));
    m_accumulated_time -= num_steps * fixed_dt;

    int max_steps = 20;
    num_steps = std::min(num_steps, max_steps);

    for (int i = 0; i < num_steps; ++i) {
        step_simulation(elapsed);
        m_island_manager.update(m_last_time + fixed_dt * i);
    }

    m_last_time = time;
}

}
