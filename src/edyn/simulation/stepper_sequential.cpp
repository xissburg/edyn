#include "edyn/simulation/stepper_sequential.hpp"
#include "edyn/collision/contact_event_emitter.hpp"
#include "edyn/context/settings.hpp"
#include "edyn/collision/broadphase.hpp"
#include "edyn/collision/contact_manifold_map.hpp"
#include "edyn/collision/narrowphase.hpp"
#include "edyn/core/entity_graph.hpp"
#include "edyn/dynamics/material_mixing.hpp"
#include "edyn/time/time.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

stepper_sequential::stepper_sequential(entt::registry &registry, bool multithreaded)
    : m_registry(&registry)
    , m_island_manager(registry)
    , m_raycast_service(registry)
    , m_poly_initializer(registry)
    , m_solver(registry)
    , m_multithreaded(multithreaded)
{
    m_last_time = performance_time();
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

    auto &bphase = m_registry->ctx().at<broadphase>();
    auto &nphase = m_registry->ctx().at<narrowphase>();
    auto &emitter = m_registry->ctx().at<contact_event_emitter>();

    m_poly_initializer.init_new_shapes();

    for (int i = 0; i < num_steps; ++i) {
        auto step_time = m_last_time + fixed_dt * i;
        bphase.update_sequential(m_multithreaded);
        m_island_manager.update(step_time);
        nphase.update_sequential(m_multithreaded);
        m_solver.update(fixed_dt);
        emitter.consume_events();
    }

    m_last_time = time;
}

void stepper_sequential::step_simulation() {
    auto fixed_dt = m_registry->ctx().at<settings>().fixed_dt;
    m_last_time = performance_time();

    auto &bphase = m_registry->ctx().at<broadphase>();
    auto &nphase = m_registry->ctx().at<narrowphase>();
    auto &emitter = m_registry->ctx().at<contact_event_emitter>();

    m_poly_initializer.init_new_shapes();
    bphase.update_sequential(m_multithreaded);
    m_island_manager.update(m_last_time);
    nphase.update_sequential(m_multithreaded);
    m_solver.update(fixed_dt);
    emitter.consume_events();
}

void stepper_sequential::set_paused(bool paused) {
    if (!paused) {
        m_last_time = performance_time();
    }
}

}
