#include "edyn/parallel/island_worker.hpp"
#include "edyn/parallel/job.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/time/time.hpp"
#include "edyn/parallel/job_dispatcher.hpp"
#include "edyn/serialization/memory_archive.hpp"
#include "edyn/comp/constraint.hpp"

namespace edyn {

void island_worker_func(job::data_type &data) {
    auto archive = memory_input_archive(data.data(), data.size());
    intptr_t worker_intptr;
    archive(worker_intptr);
    auto *worker = reinterpret_cast<island_worker *>(worker_intptr);

    if (worker->is_terminating()) {
        // `worker` is dynamically allocated and it reschedules itself after every
        // invocation to `update`. If it is marked as finished, it should be
        // deallocated thus stopping the rescheduling cycle.
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
    , m_bphase(m_registry)
    , m_nphase(m_registry)
    , m_solver(m_registry)
    , m_snapshot_builder(m_entity_map)
    , m_importing_snapshot(false)
{
    m_island_entity = m_registry.create();
    m_entity_map.insert(island_entity, m_island_entity);

    m_message_queue.sink<registry_snapshot>().connect<&island_worker::on_registry_snapshot>(*this);
    m_message_queue.sink<msg::set_paused>().connect<&island_worker::on_set_paused>(*this);
    m_message_queue.sink<msg::step_simulation>().connect<&island_worker::on_step_simulation>(*this);

    m_registry.on_construct<island_node>().connect<&island_worker::on_construct_island_node>(*this);
    m_registry.on_destroy<island_node>().connect<&island_worker::on_destroy_island_node>(*this);

    m_registry.on_construct<constraint>().connect<&island_worker::on_construct_constraint>(*this);
    m_registry.on_destroy<constraint>().connect<&island_worker::on_destroy_constraint>(*this);
}

void island_worker::on_construct_constraint(entt::registry &registry, entt::entity entity) {
    auto &con = registry.get<constraint>(entity);

    // Initialize constraint.
    std::visit([&] (auto &&c) {
        c.update(solver_stage_value_t<solver_stage::init>{}, entity, con, registry, 0);
    }, con.var);
}

void island_worker::on_destroy_constraint(entt::registry &registry, entt::entity entity) {
    auto &con = registry.get<constraint>(entity);

    // Destroy all constraint rows.
    for (size_t i = 0; i < con.num_rows; ++i) {
        registry.destroy(con.row[i]);
    }
}

void island_worker::on_construct_island_node(entt::registry &registry, entt::entity entity) {
    if (m_importing_snapshot) return;

    auto &container = registry.emplace<island_container>(entity);
    container.entities.push_back(m_island_entity);

    auto &isle = registry.get<island>(m_island_entity);
    isle.entities.push_back(entity);

    m_snapshot_builder.maybe_updated(entity, registry, all_components{});
    m_snapshot_builder.updated<island>(m_island_entity, isle);
}

void island_worker::on_destroy_island_node(entt::registry &registry, entt::entity entity) {
    if (m_importing_snapshot) return;

    auto &isle = registry.get<island>(m_island_entity);
    isle.entities.erase(
        std::remove(
            isle.entities.begin(),
            isle.entities.end(), entity),
        isle.entities.end());

    m_snapshot_builder.destroyed(entity);
    m_snapshot_builder.updated<island>(m_island_entity, isle);
}

void island_worker::on_registry_snapshot(const registry_snapshot &snapshot) {
    // Import components from main registry.
    m_importing_snapshot = true;
    snapshot.import(m_registry, m_entity_map);
    m_importing_snapshot = false;
}

void island_worker::sync() {
    auto &isle = m_registry.get<island>(m_island_entity);

    // Add transient components of procedural nodes to snapshot.
    for (auto entity : isle.entities) {
        auto &node = m_registry.get<island_node>(entity);
        // TODO: perhaps remove `island_node::procedural` and use a tag instead.
        if (node.procedural) {
            m_snapshot_builder.maybe_updated(entity, m_registry, transient_components{});
        }
    }

    auto view = m_registry.view<island_node_dirty>();
    view.each([&] (entt::entity entity, island_node_dirty &dirty) {
        m_snapshot_builder.updated(entity, m_registry, 
            dirty.indexes.begin(), dirty.indexes.end(), 
            all_components{});
    });

    m_message_queue.send<registry_snapshot>(m_snapshot_builder.get_snapshot());

    // Clear snapshot for the next run.
    m_snapshot_builder.clear();
    m_registry.clear<island_node_dirty>();
}

void island_worker::update() {
    // Process messages.
    m_message_queue.update();

    if (!m_paused) {
        auto &isle = m_registry.get<island>(m_island_entity);
        auto timestamp = (double)performance_counter() / (double)performance_frequency();
        auto dt = timestamp - isle.timestamp;

        if (dt >= m_fixed_dt) {
            step();
        }
    }

    // Reschedule this job.
    reschedule();
}

void island_worker::step() {
    m_bphase.update();
    m_nphase.update();
    m_solver.update(m_fixed_dt);

    auto &isle = m_registry.get<island>(m_island_entity);
    isle.timestamp += m_fixed_dt;

    sync();
}

void island_worker::reschedule() {
    auto j = job();
    j.func = &island_worker_func;
    auto archive = fixed_memory_output_archive(j.data.data(), j.data.size());
    auto ctx_intptr = reinterpret_cast<intptr_t>(this);
    archive(ctx_intptr);
    //job_dispatcher::global()::async_after(isle.timestamp + m_fixed_dt - timestamp, j);
    job_dispatcher::global().async(j);
}

void island_worker::on_set_paused(const msg::set_paused &msg) {
    m_paused = msg.paused;
    auto &isle = m_registry.get<island>(m_island_entity);
    auto timestamp = (double)performance_counter() / (double)performance_frequency();
    isle.timestamp = timestamp;
}

void island_worker::on_step_simulation(const msg::step_simulation &msg) {
    step();
}

bool island_worker::is_terminated() const {
    return m_terminated.load(std::memory_order_relaxed);
}

bool island_worker::is_terminating() const {
    return m_terminating.load(std::memory_order_relaxed);
}

void island_worker::terminate() {
    m_terminating.store(true, std::memory_order_relaxed);
}

void island_worker::do_terminate() {
    {
        auto lock = std::lock_guard(m_terminate_mutex);
        m_terminated.store(true, std::memory_order_relaxed);
    }
    //m_message_queue.send<msg::island_worker_terminated>();
    m_terminate_cv.notify_one();
}

void island_worker::join() {
    auto lock = std::unique_lock(m_terminate_mutex);
    m_terminate_cv.wait(lock, [&] { return is_terminated(); });
}

}