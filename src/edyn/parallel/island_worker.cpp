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
    , m_delta_builder(m_entity_map)
    , m_importing_delta(false)
    , m_splitting_island(false)
    , m_topology_changed(false)
{
    m_island_entity = m_registry.create();
    m_entity_map.insert(island_entity, m_island_entity);
    m_delta_builder.updated(m_island_entity);

    m_message_queue.sink<registry_delta>().connect<&island_worker::on_registry_delta>(*this);
    m_message_queue.sink<msg::set_paused>().connect<&island_worker::on_set_paused>(*this);
    m_message_queue.sink<msg::step_simulation>().connect<&island_worker::on_step_simulation>(*this);

    m_registry.on_construct<island_node>().connect<&island_worker::on_construct_island_node>(*this);
    m_registry.on_update<island_node>().connect<&island_worker::on_update_island_node>(*this);
    m_registry.on_destroy<island_node>().connect<&island_worker::on_destroy_island_node>(*this);

    m_registry.on_construct<constraint>().connect<&island_worker::on_construct_constraint>(*this);
    m_registry.on_destroy<constraint>().connect<&island_worker::on_destroy_constraint>(*this);
    
    m_registry.on_construct<contact_manifold>().connect<&island_worker::on_construct_contact_manifold>(*this);
}

void island_worker::on_construct_constraint(entt::registry &registry, entt::entity entity) {
    if (m_importing_delta || m_splitting_island) return;

    auto &con = registry.get<constraint>(entity);

    // Initialize constraint.
    std::visit([&] (auto &&c) {
        c.update(solver_stage_value_t<solver_stage::init>{}, entity, con, registry, 0);
    }, con.var);
}

void island_worker::on_destroy_constraint(entt::registry &registry, entt::entity entity) {
    if (m_importing_delta || m_splitting_island) return;

    auto &con = registry.get<constraint>(entity);

    // Destroy all constraint rows.
    for (size_t i = 0; i < con.row.size(); ++i) {
        auto row_entity = con.row[i];
        if (registry.valid(row_entity)) {
            registry.destroy(row_entity);
        }
    }
}

void island_worker::on_construct_contact_manifold(entt::registry &registry, entt::entity entity) {
    if (m_importing_delta) {
        m_new_imported_contact_manifolds.push_back(entity);
    }
}

void island_worker::on_construct_island_node(entt::registry &registry, entt::entity entity) {
    if (m_importing_delta || m_splitting_island) {
        return;
    }
    
    auto &container = registry.emplace<island_container>(entity);
    container.entities.push_back(m_island_entity);

    m_delta_builder.created(entity);
    m_delta_builder.maybe_updated(entity, registry, all_components{});
}

void island_worker::on_update_island_node(entt::registry &registry, entt::entity entity) {
    m_topology_changed = true;
}

void island_worker::on_destroy_island_node(entt::registry &registry, entt::entity entity) {

    if (!m_importing_delta && !m_splitting_island) {
        m_delta_builder.destroyed(entity);
    }

    // Remove from connected nodes.
    auto &node = registry.get<island_node>(entity);

    for (auto other : node.entities) {
        if (!registry.valid(other) || !registry.has<island_node>(other)) continue;
        auto &other_node = registry.get<island_node>(other);
        other_node.entities.erase(
            std::remove(
                other_node.entities.begin(),
                other_node.entities.end(), entity), 
            other_node.entities.end());
        if (!m_importing_delta && !m_splitting_island) {
            m_delta_builder.updated<island_node>(other, other_node);
        }
    }

    m_topology_changed = true;
}

void island_worker::on_registry_delta(const registry_delta &delta) {
    // Import components from main registry.
    m_importing_delta = true;
    delta.import(m_registry, m_entity_map);
    m_importing_delta = false;

    for (auto remote_entity : delta.created()) {
        if (!m_entity_map.has_rem(remote_entity)) continue;
        auto local_entity = m_entity_map.remloc(remote_entity);
        m_delta_builder.insert_entity_mapping(local_entity);
    }
}

void island_worker::sync() {
    // Add transient components of procedural nodes to delta.
    m_registry.view<procedural_tag>().each([&] (entt::entity entity) {
        m_delta_builder.maybe_updated(entity, m_registry, transient_components{});
    });

    auto view = m_registry.view<island_node_dirty>();
    view.each([&] (entt::entity entity, island_node_dirty &dirty) {
        m_delta_builder.updated(entity, m_registry,
            dirty.indexes.begin(), dirty.indexes.end(),
            all_components{});

        if (dirty.is_new_entity) {
            m_delta_builder.created(entity);
        }
    });

    m_message_queue.send<registry_delta>(m_delta_builder.get_delta());

    // Clear delta for the next run.
    m_delta_builder.clear();
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

            constexpr int max_lagging_steps = 10;
            auto num_steps = int(std::floor(dt / m_fixed_dt));

            if (num_steps > max_lagging_steps) {
                auto remainder = dt - num_steps * m_fixed_dt;
                auto &isle_time = m_registry.get<island_timestamp>(m_island_entity);
                isle_time.value = time - (remainder + max_lagging_steps * m_fixed_dt);
                m_delta_builder.updated<island_timestamp>(m_island_entity, isle_time);
            }
        }
    }

    // Reschedule this job only if not paused.
    if (!m_paused) {
        reschedule();
    }
}

void island_worker::step() {
    init_new_imported_contact_manifolds();
    m_solver.update(m_fixed_dt);
    m_bphase.update();
    m_nphase.update();

    auto &isle_time = m_registry.get<island_timestamp>(m_island_entity);
    isle_time.value += m_fixed_dt;
    m_delta_builder.updated<island_timestamp>(m_island_entity, isle_time);

    sync();

    if (m_topology_changed) {
        validate_island();
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

    auto time = (double)performance_counter() / (double)performance_frequency();
    auto &isle_time = m_registry.get<island_timestamp>(m_island_entity);
    auto delta_time = isle_time.value + m_fixed_dt - time;

    if (delta_time > 0) {
        job_dispatcher::global().async_after(delta_time, j);
    } else {
        job_dispatcher::global().async(j);
    }
}

void island_worker::init_new_imported_contact_manifolds() {
    m_nphase.update_contact_manifolds(m_new_imported_contact_manifolds.begin(),
                                      m_new_imported_contact_manifolds.end());
    m_new_imported_contact_manifolds.clear();
}

void island_worker::maybe_split_island() {
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
    auto local_entities = connected_components.front();
    m_splitting_island = true;

    std::unordered_set<entt::entity> destroyed_entities;
    m_delta_builder.split(local_entities);
    
    for (auto it = std::next(connected_components.begin()); it != connected_components.end(); ++it) {
        for (auto entity : *it) {
            // Destroy node locally if it is not contained in the local island anymore.
            auto should_destroy = local_entities.count(entity) == 0;

            if (should_destroy) {
                destroyed_entities.insert(entity);

                // Check if it's valid since it could have already been destroyed
                // in case it is a child of another entity that was destroyed
                // earlier in this loop (e.g. a constraint will destroy its rows).
                if (m_registry.valid(entity)) {
                    m_registry.destroy(entity);
                }
            }
        }

        m_delta_builder.split(*it);
    }

    for (auto entity : destroyed_entities) {
        if (m_entity_map.has_loc(entity)) {
            m_entity_map.erase_loc(entity);
        }
    }

    m_splitting_island = false;

    sync();
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

void island_worker::validate_island() {
    const auto &node_view = m_registry.view<const island_node>();

    // All siblings of a node should point back to itself.
    for (entt::entity entity : node_view) {
        auto &node = node_view.get(entity);
        for (auto other : node.entities) {
            auto &other_node = node_view.get(other);
            if (std::find(other_node.entities.begin(), 
                          other_node.entities.end(), 
                          entity) == other_node.entities.end()) {
                EDYN_ASSERT(false);
            }
        }
    }

    auto container_view = m_registry.view<const island_container>();

    // All island containers shoud contain the local island.
    for (entt::entity entity : container_view) {
        auto &container = container_view.get(entity);

        if (std::find(container.entities.begin(), 
                      container.entities.end(), 
                      m_island_entity) == container.entities.end()) {
            EDYN_ASSERT(false);
        }

        if (container.entities.size() != 1) {
            EDYN_ASSERT(false);
        }
    }
}

}