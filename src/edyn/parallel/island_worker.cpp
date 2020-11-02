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
    , m_splitting_island(false)
    , m_topology_changed(false)
{
    m_island_entity = m_registry.create();
    m_entity_map.insert(island_entity, m_island_entity);

    m_message_queue.sink<registry_snapshot>().connect<&island_worker::on_registry_snapshot>(*this);
    m_message_queue.sink<msg::set_paused>().connect<&island_worker::on_set_paused>(*this);
    m_message_queue.sink<msg::step_simulation>().connect<&island_worker::on_step_simulation>(*this);

    m_registry.on_construct<island_node>().connect<&island_worker::on_construct_island_node>(*this);
    m_registry.on_update<island_node>().connect<&island_worker::on_update_island_node>(*this);
    m_registry.on_destroy<island_node>().connect<&island_worker::on_destroy_island_node>(*this);

    m_registry.on_construct<constraint>().connect<&island_worker::on_construct_constraint>(*this);
    m_registry.on_destroy<constraint>().connect<&island_worker::on_destroy_constraint>(*this);
}

void island_worker::on_construct_constraint(entt::registry &registry, entt::entity entity) {
    if (m_importing_snapshot || m_splitting_island) return;

    auto &con = registry.get<constraint>(entity);

    // Initialize constraint.
    std::visit([&] (auto &&c) {
        c.update(solver_stage_value_t<solver_stage::init>{}, entity, con, registry, 0);
    }, con.var);
}

void island_worker::on_destroy_constraint(entt::registry &registry, entt::entity entity) {
    if (m_importing_snapshot || m_splitting_island) return;

    auto &con = registry.get<constraint>(entity);

    // Destroy all constraint rows.
    for (size_t i = 0; i < con.num_rows; ++i) {
        registry.destroy(con.row[i]);
    }
}

void island_worker::on_construct_island_node(entt::registry &registry, entt::entity entity) {
    if (m_importing_snapshot || m_splitting_island) return;
    
    auto &container = registry.emplace<island_container>(entity);
    container.entities.push_back(m_island_entity);

    auto &isle = registry.get<island>(m_island_entity);
    isle.entities.push_back(entity);

    m_snapshot_builder.maybe_updated(entity, registry, all_components{});
    m_snapshot_builder.updated<island>(m_island_entity, isle);
}

void island_worker::on_update_island_node(entt::registry &registry, entt::entity entity) {
    //m_topology_changed = true;
}

void island_worker::on_destroy_island_node(entt::registry &registry, entt::entity entity) {
    if (m_importing_snapshot || m_splitting_island) return;

    auto &isle = registry.get<island>(m_island_entity);
    isle.entities.erase(
        std::remove(
            isle.entities.begin(),
            isle.entities.end(), entity),
        isle.entities.end());

    m_snapshot_builder.destroyed(entity);
    m_snapshot_builder.updated<island>(m_island_entity, isle);

    // Remove from connected nodes.
    auto &node = registry.get<island_node>(entity);

    for (auto other : node.entities) {
        auto &other_node = registry.get<island_node>(other);
        other_node.entities.erase(
            std::remove(
                other_node.entities.begin(),
                other_node.entities.end(), entity), 
            other_node.entities.end());
        m_snapshot_builder.updated<island_node>(other, other_node);
    }

    m_topology_changed = true;
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
        if (m_registry.has<procedural_tag>(entity)) {
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
        auto &isle_time = m_registry.get<island_timestamp>(m_island_entity);
        auto time = (double)performance_counter() / (double)performance_frequency();
        auto dt = time - isle_time.value;

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

    auto &isle_time = m_registry.get<island_timestamp>(m_island_entity);
    isle_time.value += m_fixed_dt;
    m_snapshot_builder.updated<island_timestamp>(m_island_entity, isle_time);

    sync();

    if (m_topology_changed) {
        maybe_split_island();
        m_topology_changed = false;
    }
}

void island_worker::reschedule() {
    auto j = job();
    j.func = &island_worker_func;
    auto archive = fixed_memory_output_archive(j.data.data(), j.data.size());
    auto ctx_intptr = reinterpret_cast<intptr_t>(this);
    archive(ctx_intptr);
    //job_dispatcher::global()::async_after(isle_time.value + m_fixed_dt - timestamp, j);
    job_dispatcher::global().async(j);
}

void island_worker::maybe_split_island() {
    // Start from any node.
    auto node_view = m_registry.view<island_node>();
    if (node_view.empty()) return;

    std::vector<entt::entity> node_entities;
    node_entities.reserve(node_view.size());
    m_registry.view<procedural_tag>().each([&node_entities] (entt::entity entity) {
        node_entities.push_back(entity);
    });

    std::vector<std::unordered_set<entt::entity>> connected_components;

    while (!node_entities.empty()) {
        std::unordered_set<entt::entity> connected;
        std::vector<entt::entity> to_visit;

        auto node_entity = node_entities.back();
        to_visit.push_back(node_entity);

        while (!to_visit.empty()) {
            auto entity = to_visit.back();
            to_visit.pop_back();

            // Add to connected component.
            connected.insert(entity);

            // Remove from main set.
            node_entities.erase(
                std::remove(
                    node_entities.begin(), 
                    node_entities.end(), entity),
                node_entities.end());

            // Add related entities to be visited next.
            auto &curr_node = node_view.get(entity);

            for (auto other : curr_node.entities) {
                auto already_visited = std::find(
                    connected.begin(), connected.end(), other) != connected.end();
                if (already_visited) continue;

                if (m_registry.has<procedural_tag>(other)) {
                    to_visit.push_back(other);
                } else {
                    connected.insert(other);
                }
            }
        }

        connected_components.push_back(connected);
    }

    if (connected_components.size() == 1) return;

    // There's more than one island in this worker which means the work can now
    // be split and run in parallel in two or more workers.
    // Keep the first connected component in this island.
    auto &isle = m_registry.get<island>(m_island_entity);
    isle.entities = std::vector<entt::entity>(connected_components.front().begin(),
                                              connected_components.front().end());
    auto builder = registry_snapshot_builder(m_entity_map);
    builder.updated<island>(m_island_entity, isle);
    m_message_queue.send<registry_snapshot>(builder.get_snapshot());

    m_splitting_island = true;

    for (auto it = std::next(connected_components.begin()); it != connected_components.end(); ++it) {
        for (auto ent_it = it->begin(); ent_it != it->end(); ++ent_it) {
            auto entity = *ent_it;

            // Destroy node locally if it is not contained in the local island anymore.
            auto should_destroy = 
                std::find(isle.entities.begin(), isle.entities.end(), entity) == isle.entities.end();

            if (should_destroy) {
                if (m_entity_map.has_loc(entity)) {
                    m_entity_map.erase_loc(entity);
                }

                if (m_registry.valid(entity)) {
                    m_registry.destroy(entity);
                }
            }
        }
    }

    m_splitting_island = false;

    m_message_queue.send<msg::split_island>(std::vector<std::unordered_set<entt::entity>>(std::next(connected_components.begin()), connected_components.end()));
}

void island_worker::on_set_paused(const msg::set_paused &msg) {
    m_paused = msg.paused;
    auto &isle_time = m_registry.get<island_timestamp>(m_island_entity);
    auto timestamp = (double)performance_counter() / (double)performance_frequency();
    isle_time.value = timestamp;
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