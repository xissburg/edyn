#include "edyn/simulation/stepper_sequential.hpp"
#include "edyn/collision/contact_event_emitter.hpp"
#include "edyn/context/settings.hpp"
#include "edyn/collision/broadphase.hpp"
#include "edyn/collision/contact_manifold_map.hpp"
#include "edyn/collision/narrowphase.hpp"
#include "edyn/core/entity_graph.hpp"
#include "edyn/dynamics/material_mixing.hpp"
#include "edyn/networking/sys/decay_discontinuities.hpp"
#include "edyn/time/time.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

stepper_sequential::stepper_sequential(entt::registry &registry, bool multithreaded)
    : m_registry(&registry)
    , m_island_manager(registry)
    , m_poly_initializer(registry)
    , m_solver(registry)
    , m_multithreaded(multithreaded)
    , m_paused(false)
{
    m_last_time = performance_time();
    m_island_manager.set_last_time(m_last_time);
}

void stepper_sequential::update() {
    if (m_paused) {
        m_island_manager.update(m_last_time);
        return;
    }

    auto time = performance_time();
    auto elapsed = time - m_last_time;

    auto fixed_dt = m_registry->ctx().at<settings>().fixed_dt;
    m_accumulated_time += elapsed;
    auto num_steps = static_cast<unsigned>(std::floor(m_accumulated_time / fixed_dt));
    m_accumulated_time -= num_steps * fixed_dt;

    auto &bphase = m_registry->ctx().at<broadphase>();
    auto &nphase = m_registry->ctx().at<narrowphase>();
    auto &emitter = m_registry->ctx().at<contact_event_emitter>();
    auto &settings = m_registry->ctx().at<edyn::settings>();

    num_steps = std::min(num_steps, settings.max_steps_per_update);

    m_poly_initializer.init_new_shapes();

    for (int i = 0; i < num_steps; ++i) {
        auto step_time = m_last_time + fixed_dt * i;

        if (settings.pre_step_callback) {
            (*settings.pre_step_callback)(*m_registry);
        }

        bphase.update(m_multithreaded);
        m_island_manager.update(step_time);
        nphase.update(m_multithreaded);
        m_solver.update(m_multithreaded);
        emitter.consume_events();
        decay_discontinuities(*m_registry);

        if (settings.clear_actions_func) {
            (*settings.clear_actions_func)(*m_registry);
        }

        if (settings.post_step_callback) {
            (*settings.post_step_callback)(*m_registry);
        }
    }

    m_last_time = time;
}

void stepper_sequential::step_simulation() {
    EDYN_ASSERT(m_paused);

    m_last_time = performance_time();

    auto &bphase = m_registry->ctx().at<broadphase>();
    auto &nphase = m_registry->ctx().at<narrowphase>();
    auto &emitter = m_registry->ctx().at<contact_event_emitter>();
    auto &settings = m_registry->ctx().at<edyn::settings>();

    if (settings.pre_step_callback) {
        (*settings.pre_step_callback)(*m_registry);
    }

    m_poly_initializer.init_new_shapes();
    bphase.update(m_multithreaded);
    m_island_manager.update(m_last_time);
    nphase.update(m_multithreaded);
    m_solver.update(m_multithreaded);
    emitter.consume_events();
    decay_discontinuities(*m_registry);

    if (settings.clear_actions_func) {
        (*settings.clear_actions_func)(*m_registry);
    }

    if (settings.post_step_callback) {
        (*settings.post_step_callback)(*m_registry);
    }
}

void stepper_sequential::set_paused(bool paused) {
    m_paused = paused;
    m_accumulated_time = 0;

    if (!paused) {
        m_last_time = performance_time();
    }
}

}
