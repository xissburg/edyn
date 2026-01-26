#include "edyn/simulation/stepper_sequential.hpp"
#include "edyn/context/profile.hpp"
#include "edyn/context/settings.hpp"
#include "edyn/collision/broadphase.hpp"
#include "edyn/collision/contact_manifold_map.hpp"
#include "edyn/collision/narrowphase.hpp"
#include "edyn/util/profile_util.hpp"
#include "edyn/core/entity_graph.hpp"
#include "edyn/dynamics/material_mixing.hpp"
#include "edyn/sys/update_presentation.hpp"
#include <entt/entity/registry.hpp>
#include <cstdint>

namespace edyn {

stepper_sequential::stepper_sequential(entt::registry &registry, double time, bool multithreaded)
    : m_registry(&registry)
    , m_island_manager(registry)
    , m_poly_initializer(registry)
    , m_solver(registry)
    , m_multithreaded(multithreaded)
    , m_paused(false)
    , m_last_time(time)
{
    m_island_manager.set_last_time(m_last_time);
}

void stepper_sequential::update(double time) {
#ifndef EDYN_DISABLE_PROFILING
    auto &profile = m_registry->ctx().get<profile_timers>();
    if (!m_paused) {
        profile = {};
    }
#endif

    EDYN_PROFILE_BEGIN(prof_time);

    if (m_paused) {
        m_island_manager.update(m_last_time);
        EDYN_PROFILE_MEASURE(prof_time, profile, islands);
        snap_presentation(*m_registry);
        return;
    }

    auto sim_time = get_simulation_timestamp();
    auto elapsed = std::max(time - m_last_time, 0.0);
    m_accumulated_time += elapsed;

    auto &settings = m_registry->ctx().get<edyn::settings>();
    const auto fixed_dt = settings.fixed_dt;
    const auto num_steps = static_cast<uint64_t>(std::floor(m_accumulated_time / fixed_dt));
    auto advance_dt = static_cast<double>(num_steps) * fixed_dt;
    m_accumulated_time -= advance_dt;

    auto &bphase = m_registry->ctx().get<broadphase>();
    auto &nphase = m_registry->ctx().get<narrowphase>();

    auto effective_steps = num_steps;
    auto step_dt = fixed_dt;

    if (effective_steps > settings.max_steps_per_update) {
        effective_steps = settings.max_steps_per_update;
        // Scale up the effective delta time in each step.
        step_dt = advance_dt / effective_steps;
    }

    // Initialize new AABBs and shapes even in case num_steps is zero.
    m_poly_initializer.init_new_shapes();
    bphase.init_new_aabb_entities();

    for (unsigned i = 0; i < effective_steps; ++i) {
        EDYN_PROFILE_BEGIN(step_prof_time);

        auto step_time = sim_time + step_dt * i;

        if (settings.pre_step_callback) {
            (*settings.pre_step_callback)(*m_registry);
        }

        EDYN_PROFILE_BEGIN(task_time);

        bphase.update(m_multithreaded);
        EDYN_PROFILE_MEASURE_ACCUM(task_time, profile, broadphase);

        nphase.update(m_multithreaded);
        EDYN_PROFILE_MEASURE_ACCUM(task_time, profile, narrowphase);

        m_island_manager.update(step_time);
        EDYN_PROFILE_MEASURE_ACCUM(task_time, profile, islands);

        m_solver.update(m_multithreaded);

        if (settings.clear_actions_func) {
            (*settings.clear_actions_func)(*m_registry);
        }

        if (settings.post_step_callback) {
            (*settings.post_step_callback)(*m_registry);
        }

        EDYN_PROFILE_MEASURE_ACCUM(step_prof_time, profile, step);
    }

    m_last_time = time;
    update_presentation(*m_registry, get_simulation_timestamp(), time, elapsed, fixed_dt);

#ifndef EDYN_DISABLE_PROFILING
    if (effective_steps > 0) {
        EDYN_PROFILE_MEASURE_AVG(profile, broadphase,    effective_steps);
        EDYN_PROFILE_MEASURE_AVG(profile, islands,       effective_steps);
        EDYN_PROFILE_MEASURE_AVG(profile, narrowphase,   effective_steps);
        EDYN_PROFILE_MEASURE_AVG(profile, restitution,   effective_steps);
        EDYN_PROFILE_MEASURE_AVG(profile, prepare_constraints, effective_steps);
        EDYN_PROFILE_MEASURE_AVG(profile, solve_islands, effective_steps);
        EDYN_PROFILE_MEASURE_AVG(profile, apply_results, effective_steps);
        EDYN_PROFILE_MEASURE_AVG(profile, step,          effective_steps);
    }
#endif
}

void stepper_sequential::step_simulation(double time) {
    EDYN_ASSERT(m_paused);

    m_last_time = time;

    auto &bphase = m_registry->ctx().get<broadphase>();
    auto &nphase = m_registry->ctx().get<narrowphase>();
    auto &settings = m_registry->ctx().get<edyn::settings>();

    if (settings.pre_step_callback) {
        (*settings.pre_step_callback)(*m_registry);
    }

    m_poly_initializer.init_new_shapes();
    bphase.update(m_multithreaded);
    nphase.update(m_multithreaded);
    m_island_manager.update(m_last_time);
    m_solver.update(m_multithreaded);

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
}

}
