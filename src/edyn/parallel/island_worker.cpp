#include "edyn/parallel/island_worker.hpp"
#include "edyn/config/config.h"
#include "edyn/parallel/job.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/time/time.hpp"
#include "edyn/parallel/job_dispatcher.hpp"
#include "edyn/parallel/message.hpp"
#include "edyn/serialization/memory_archive.hpp"
#include "edyn/comp/constraint.hpp"
#include "edyn/comp/dirty.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/util/island_util.hpp"
#include "edyn/collision/tree_view.hpp"
#include "edyn/parallel/external_system.hpp"
#include "edyn/parallel/graph.hpp"

namespace edyn {

void island_worker_func(job::data_type &data) {
    auto archive = memory_input_archive(data.data(), data.size());
    intptr_t worker_intptr;
    archive(worker_intptr);
    auto *worker = reinterpret_cast<island_worker *>(worker_intptr);

    if (worker->is_terminating()) {
        // `worker` is dynamically allocated and must be manually deallocated
        // when it terminates.
        worker->do_terminate();
        delete worker;
    } else {
        worker->update();
    }
}

island_worker::island_worker(entt::entity island_entity, scalar fixed_dt, message_queue_in_out message_queue)
    : m_message_queue(message_queue)
    , m_fixed_dt(fixed_dt)
    , m_paused(false)
    , m_state(state::init)
    , m_bphase(m_registry)
    , m_nphase(m_registry)
    , m_solver(m_registry)
    , m_delta_builder(make_island_delta_builder(m_entity_map))
    , m_importing_delta(false)
    , m_topology_changed(false)
    , m_pending_split_calculation(false)
    , m_calculate_split_delay(1.1)
    , m_calculate_split_timestamp(0)
    , m_number_of_connected_components(1)
{
    m_registry.set<graph>();

    m_island_entity = m_registry.create();
    m_entity_map.insert(island_entity, m_island_entity);

    m_this_job.func = &island_worker_func;
    auto archive = fixed_memory_output_archive(m_this_job.data.data(), m_this_job.data.size());
    auto ctx_intptr = reinterpret_cast<intptr_t>(this);
    archive(ctx_intptr);
}

island_worker::~island_worker() = default;

void island_worker::init() {
    m_delta_builder->insert_entity_mapping(m_island_entity);

    // Destroy children when parents are destroyed.
    m_registry.on_destroy<island_node_parent>().connect<&island_worker::on_destroy_island_node_parent>(*this);

    m_registry.on_construct<island_node>().connect<&island_worker::on_construct_island_node>(*this);
    m_registry.on_update<island_node>().connect<&island_worker::on_update_island_node>(*this);
    m_registry.on_destroy<island_node>().connect<&island_worker::on_destroy_island_node>(*this);

    m_registry.on_construct<island_container>().connect<&island_worker::on_construct_island_container>(*this);
    m_registry.on_destroy<island_container>().connect<&island_worker::on_destroy_island_container>(*this);

    m_registry.on_construct<constraint>().connect<&island_worker::on_construct_constraint>(*this);
    
    m_registry.on_construct<contact_manifold>().connect<&island_worker::on_construct_contact_manifold>(*this);

    m_message_queue.sink<island_delta>().connect<&island_worker::on_island_delta>(*this);
    m_message_queue.sink<msg::set_paused>().connect<&island_worker::on_set_paused>(*this);
    m_message_queue.sink<msg::step_simulation>().connect<&island_worker::on_step_simulation>(*this);
    m_message_queue.sink<msg::wake_up_island>().connect<&island_worker::on_wake_up_island>(*this);

    process_messages();

    if (g_external_system_init) {
        (*g_external_system_init)(m_registry);
    }

    // Assign tree view containing the updated broad-phase tree.
    m_bphase.update();
    auto tview = m_bphase.view();
    m_registry.emplace<tree_view>(m_island_entity, tview);
    m_delta_builder->created(m_island_entity, tview);

    // Sync components that were created/updated during initialization
    // including the updated `tree_view` from above.
    sync();

    m_state = state::step;
}

void island_worker::on_construct_constraint(entt::registry &registry, entt::entity entity) {
    if (m_importing_delta) return;

    auto &con = registry.get<constraint>(entity);

    // Initialize constraint.
    std::visit([&] (auto &&c) {
        c.update(solver_stage_value_t<solver_stage::init>{}, entity, con, registry, 0);
    }, con.var);
}

void island_worker::on_construct_contact_manifold(entt::registry &registry, entt::entity entity) {
    if (m_importing_delta) {
        m_new_imported_contact_manifolds.push_back(entity);
    }
}

void island_worker::on_destroy_island_node_parent(entt::registry &registry, entt::entity entity) {
    if (m_importing_delta) return;
    edyn::on_destroy_island_node_parent(registry, entity);
}

void island_worker::on_construct_island_node(entt::registry &registry, entt::entity entity) {

}

void island_worker::on_update_island_node(entt::registry &registry, entt::entity entity) {
    m_topology_changed = true;
}

void island_worker::on_destroy_island_node(entt::registry &registry, entt::entity entity) {
    // Remove from connected nodes.
    auto &node = registry.get<island_node>(entity);

    for (auto other : node.entities) {
        auto &other_node = registry.get<island_node>(other);
        other_node.entities.erase(entity);
        if (!m_importing_delta) {
            m_delta_builder->updated<island_node>(other, other_node);
        }
    }

    m_topology_changed = true;
}

void island_worker::on_construct_island_container(entt::registry &registry, entt::entity entity) {
    if (m_importing_delta) return;
    
    auto &container = registry.get<island_container>(entity);
    container.entities.insert(m_island_entity);
    m_delta_builder->created<island_container>(entity, container);
}

void island_worker::on_destroy_island_container(entt::registry &registry, entt::entity entity) {
    if (m_importing_delta) return;
    m_delta_builder->destroyed(entity);
}

void island_worker::on_island_delta(const island_delta &delta) {
    // Import components from main registry.
    m_importing_delta = true;
    delta.import(m_registry, m_entity_map);
    m_importing_delta = false;

    for (auto remote_entity : delta.created_entities()) {
        if (!m_entity_map.has_rem(remote_entity)) continue;
        auto local_entity = m_entity_map.remloc(remote_entity);
        m_delta_builder->insert_entity_mapping(local_entity);
    }
}

void island_worker::on_wake_up_island(const msg::wake_up_island &) {
    if (!m_registry.has<sleeping_tag>(m_island_entity)) return;

    auto builder = make_island_delta_builder(m_entity_map);

    auto &isle_timestamp = m_registry.get<island_timestamp>(m_island_entity);
    isle_timestamp.value = (double)performance_counter() / (double)performance_frequency();
    builder->updated(m_island_entity, isle_timestamp);

    m_registry.view<sleeping_tag>().each([&] (entt::entity entity) {
        builder->destroyed<sleeping_tag>(entity);
    });
    m_registry.clear<sleeping_tag>();

    auto delta = builder->finish();
    m_message_queue.send<island_delta>(std::move(delta));
}

void island_worker::sync() {
    // Always update AABBs since they're needed for broad-phase in the coordinator.
    m_registry.view<AABB>().each([&] (entt::entity entity, AABB &aabb) {
        m_delta_builder->updated(entity, aabb);
    });

    // Update continuous components.
    m_registry.view<continuous>().each([&] (entt::entity entity, continuous &cont) {
        m_delta_builder->updated(entity, m_registry,
            cont.types.begin(), cont.types.end());
    });

    // Update dirty components.
    m_registry.view<dirty>().each([&] (entt::entity entity, dirty &dirty) {
        if (dirty.is_new_entity) {
            m_delta_builder->created(entity);
        }

        m_delta_builder->created(entity, m_registry,
            dirty.created_indexes.begin(), dirty.created_indexes.end());
        m_delta_builder->updated(entity, m_registry,
            dirty.updated_indexes.begin(), dirty.updated_indexes.end());
        m_delta_builder->destroyed(entity,
            dirty.destroyed_indexes.begin(), dirty.destroyed_indexes.end());
    });

    auto delta = m_delta_builder->finish();
    m_message_queue.send<island_delta>(std::move(delta));

    m_registry.clear<dirty>();
}

void island_worker::update() {
    switch (m_state) {
    case state::init:
        init();
        maybe_reschedule();
        break;
    case state::step:
        process_messages();

        if (should_step()) {
            begin_step();
            run_solver();
            if (run_broadphase()) {
                if (run_narrowphase()) {
                    finish_step();
                    maybe_reschedule();
                }
            }
        } else {
            maybe_reschedule();
        }

        break;
    case state::begin_step:
        begin_step();
        reschedule_now();
        break;
    case state::solve:
        run_solver();
        reschedule_now();
        break;
    case state::broadphase:
        if (run_broadphase()) {
            reschedule_now();
        }
        break;
    case state::broadphase_async:
        finish_broadphase();
        if (run_narrowphase()) {
            finish_step();
            maybe_reschedule();
        }
        break;
    case state::narrowphase:
        if (run_narrowphase()) {
            finish_step();
            maybe_reschedule();
        }
        break;
    case state::narrowphase_async:
        finish_narrowphase();
        finish_step();
        maybe_reschedule();
        break;
    case state::finish_step:
        finish_step();
        maybe_reschedule();
        break;
    }
}

void island_worker::process_messages() {
    m_message_queue.update();
}

bool island_worker::should_step() {
    auto time = (double)performance_counter() / (double)performance_frequency();

    if (m_state == state::begin_step) {
        m_step_start_time = time;
        return true;
    }

    if (m_paused || m_registry.has<sleeping_tag>(m_island_entity)) {
        return false;
    }

    auto &isle_time = m_registry.get<island_timestamp>(m_island_entity);
    auto dt = time - isle_time.value;

    if (dt < m_fixed_dt) {
        return false;
    }

    m_step_start_time = time;
    m_state = state::begin_step;

    return true;
}

void island_worker::begin_step() {
    EDYN_ASSERT(m_state == state::begin_step);
    if (g_external_system_pre_step) {
        (*g_external_system_pre_step)(m_registry);
    }

    init_new_imported_contact_manifolds();

    m_state = state::solve;
}

void island_worker::run_solver() {
    EDYN_ASSERT(m_state == state::solve);
    m_solver.update(m_fixed_dt);
    m_state = state::broadphase;
}

bool island_worker::run_broadphase() {
    EDYN_ASSERT(m_state == state::broadphase);

    if (m_bphase.parallelizable()) {
        m_state = state::broadphase_async;
        m_bphase.update_async(m_this_job);
        return false;
    } else {
        m_bphase.update();
        m_state = state::narrowphase;
        return true;
    }
}

void island_worker::finish_broadphase() {
    EDYN_ASSERT(m_state == state::broadphase_async);
    m_bphase.finish_async_update();
    m_state = state::narrowphase;
}

bool island_worker::run_narrowphase() {
    EDYN_ASSERT(m_state == state::narrowphase);

    if (m_nphase.parallelizable()) {
        m_state = state::narrowphase_async;
        m_nphase.update_async(m_this_job);
        return false;
    } else {
        m_nphase.update();
        m_state = state::finish_step;
        return true;
    }
}

void island_worker::finish_narrowphase() {
    EDYN_ASSERT(m_state == state::narrowphase_async);
    m_nphase.finish_async_update();
    m_state = state::finish_step;
}

void island_worker::finish_step() {
    EDYN_ASSERT(m_state == state::finish_step);

    auto &isle_time = m_registry.get<island_timestamp>(m_island_entity);
    auto dt = m_step_start_time - isle_time.value;

    // Set a limit on the number of steps the worker can lag behind the current
    // time to prevent it from getting stuck in the past in case of a
    // substantial slowdown.
    constexpr int max_lagging_steps = 10;
    auto num_steps = int(std::floor(dt / m_fixed_dt));

    if (num_steps > max_lagging_steps) {
        auto remainder = dt - num_steps * m_fixed_dt;
        isle_time.value = m_step_start_time - (remainder + max_lagging_steps * m_fixed_dt);
    } else {
        isle_time.value += m_fixed_dt;
    }

    m_delta_builder->updated<island_timestamp>(m_island_entity, isle_time);

    // Update tree view.
    auto tview = m_bphase.view();
    m_registry.replace<tree_view>(m_island_entity, tview);
    m_delta_builder->updated(m_island_entity, tview);

    maybe_go_to_sleep();

    if (m_topology_changed) {
        auto time = (double)performance_counter() / (double)performance_frequency();

        if (m_pending_split_calculation) {
            if (time - m_calculate_split_timestamp > m_calculate_split_delay) {
                m_pending_split_calculation = false;

                // If the graph has more than one connected component, it means
                // this island could be split.
                if (!m_registry.ctx<graph>().is_single_connected_component()) {
                    m_message_queue.send<msg::split_island>();
                }
                m_topology_changed = false;
            }            
        } else {
            m_pending_split_calculation = true;
            m_calculate_split_timestamp = time;
        }
    }

    if (g_external_system_post_step) {
        (*g_external_system_post_step)(m_registry);
    }

    sync();

    m_state = state::step;
}

void island_worker::reschedule_now() {
    job_dispatcher::global().async(m_this_job);
}

void island_worker::maybe_reschedule() {
    // Reschedule this job only if not paused nor sleeping.
    auto sleeping = m_registry.has<sleeping_tag>(m_island_entity);
    auto paused = m_paused;

    // The update is done and this job can be rescheduled after this point
    auto reschedule_count = m_reschedule_counter.exchange(0, std::memory_order_acq_rel);
    EDYN_ASSERT(reschedule_count != 0);

    // If the number of reschedule requests is greater than one, it means there
    // are external requests involved, not just the normal internal reschedule.
    // Always reschedule for immediate execution in that case
    if (reschedule_count == 1) {
        if (!paused && !sleeping) {
            reschedule_later();
        }
    } else {
        reschedule();
    }
}

void island_worker::reschedule_later() {
    // Only reschedule if it has not been scheduled and updated already.
    auto reschedule_count = m_reschedule_counter.fetch_add(1, std::memory_order_acq_rel);
    if (reschedule_count > 0) return;

    // If the timestamp of the current registry state is more that `m_fixed_dt`
    // before the current time, schedule it to run at a later time.
    auto time = (double)performance_counter() / (double)performance_frequency();
    auto &isle_time = m_registry.get<island_timestamp>(m_island_entity);
    auto delta_time = isle_time.value + m_fixed_dt - time;

    if (delta_time > 0) {
        job_dispatcher::global().async_after(delta_time, m_this_job);
    } else {
        job_dispatcher::global().async(m_this_job);
    }
}

void island_worker::reschedule() {
    // Only reschedule if it has not been scheduled and updated already.
    auto reschedule_count = m_reschedule_counter.fetch_add(1, std::memory_order_acq_rel);
    if (reschedule_count > 0) return;

    job_dispatcher::global().async(m_this_job);
}

void island_worker::init_new_imported_contact_manifolds() {
    // Find contact points for new manifolds imported from the main registry.
    m_nphase.update_contact_manifolds(m_new_imported_contact_manifolds.begin(),
                                      m_new_imported_contact_manifolds.end());
    m_new_imported_contact_manifolds.clear();
}

void island_worker::maybe_go_to_sleep() {
    if (could_go_to_sleep()) {
        const auto &isle_time = m_registry.get<island_timestamp>(m_island_entity);

        if (!m_sleep_timestamp) {
            m_sleep_timestamp = isle_time.value;
        } else {
            auto sleep_dt = isle_time.value - *m_sleep_timestamp;
            if (sleep_dt > island_time_to_sleep) {
                go_to_sleep();
                m_sleep_timestamp.reset();
            }
        }
    } else {
        m_sleep_timestamp.reset();
    }
}

bool island_worker::could_go_to_sleep() {
    // If any entity has a `sleeping_disabled_tag` then the island should
    // not go to sleep, since the movement of all entities depend on one
    // another in the same island.
    if (!m_registry.view<sleeping_disabled_tag>().empty()) {
        return false;
    }

    // Check if there are any entities moving faster than the sleep threshold.
    auto vel_view = m_registry.view<linvel, angvel, procedural_tag>();
    for (auto entity : vel_view) {
        auto [v, w] = vel_view.get<linvel, angvel>(entity);

        if ((length_sqr(v) > island_linear_sleep_threshold * island_linear_sleep_threshold) || 
            (length_sqr(w) > island_angular_sleep_threshold * island_angular_sleep_threshold)) {
            return false;
        }
    }

    return true;
}

void island_worker::go_to_sleep() {
    m_registry.emplace<sleeping_tag>(m_island_entity);
    m_delta_builder->created(m_island_entity, sleeping_tag{});

    // Assign `sleeping_tag` to all procedural entities.
    m_registry.view<procedural_tag>().each([&] (entt::entity entity) {
        if (auto *v = m_registry.try_get<linvel>(entity); v) {
            *v = vector3_zero;
            m_delta_builder->updated(entity, *v);
        }

        if (auto *w = m_registry.try_get<angvel>(entity); w) {
            *w = vector3_zero;
            m_delta_builder->updated(entity, *w);
        }

        m_registry.emplace<sleeping_tag>(entity);
        m_delta_builder->created(entity, sleeping_tag{});
    });
}

void island_worker::on_set_paused(const msg::set_paused &msg) {
    m_paused = msg.paused;
    auto &isle_time = m_registry.get<island_timestamp>(m_island_entity);
    auto timestamp = (double)performance_counter() / (double)performance_frequency();
    isle_time.value = timestamp;
}

void island_worker::on_step_simulation(const msg::step_simulation &) {
    if (!m_registry.has<sleeping_tag>(m_island_entity)) {
        m_state = state::begin_step;
    }
}

bool island_worker::is_terminated() const {
    return m_terminated.load(std::memory_order_acquire);
}

bool island_worker::is_terminating() const {
    return m_terminating.load(std::memory_order_acquire);
}

void island_worker::terminate() {
    m_terminating.store(true, std::memory_order_release);
    reschedule();
}

void island_worker::do_terminate() {
    {
        auto lock = std::lock_guard(m_terminate_mutex);
        m_terminated.store(true, std::memory_order_release);
    }
    m_terminate_cv.notify_one();
}

void island_worker::join() {
    auto lock = std::unique_lock(m_terminate_mutex);
    m_terminate_cv.wait(lock, [&] { return is_terminated(); });
}

}
