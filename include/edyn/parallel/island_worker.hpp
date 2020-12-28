#ifndef EDYN_PARALLEL_ISLAND_WORKER_HPP
#define EDYN_PARALLEL_ISLAND_WORKER_HPP

#include <mutex>
#include <atomic>
#include <optional>
#include <entt/entt.hpp>
#include <condition_variable>
#include "edyn/parallel/job.hpp"
#include "edyn/dynamics/solver.hpp"
#include "edyn/parallel/message.hpp"
#include "edyn/collision/narrowphase.hpp"
#include "edyn/parallel/message_queue.hpp"
#include "edyn/parallel/registry_delta.hpp"
#include "edyn/collision/broadphase_worker.hpp"

namespace edyn {

void island_worker_func(job::data_type &);

/**
 * Simulates one island in a worker thread independently.
 */
class island_worker final {

    void reschedule_later();
    void calculate_topology();
    void do_terminate();
    void validate_island();
    void init_new_imported_contact_manifolds();
    void maybe_go_to_sleep();
    bool could_go_to_sleep();
    void go_to_sleep();

public:
    island_worker(entt::entity island_entity, scalar fixed_dt, message_queue_in_out message_queue);

    ~island_worker();

    void on_registry_delta(const registry_delta &delta);

    void sync();

    void update();

    void step();

    void reschedule();

    void on_construct_constraint(entt::registry &, entt::entity);

    void on_construct_contact_manifold(entt::registry &, entt::entity);

    void on_destroy_island_node_parent(entt::registry &, entt::entity);
    
    void on_construct_island_node(entt::registry &, entt::entity);
    void on_update_island_node(entt::registry &, entt::entity);
    void on_destroy_island_node(entt::registry &, entt::entity);

    void on_construct_island_container(entt::registry &, entt::entity);
    void on_destroy_island_container(entt::registry &, entt::entity);

    void on_set_paused(const msg::set_paused &msg);
    void on_step_simulation(const msg::step_simulation &msg);
    void on_wake_up_island(const msg::wake_up_island &);

    bool is_terminated() const;
    bool is_terminating() const;
    void terminate();
    void join();

    friend void island_worker_func(job::data_type &);

private:
    entt::registry m_registry;
    entt::entity m_island_entity;
    entity_map m_entity_map;
    broadphase_worker m_bphase;
    narrowphase m_nphase;
    solver m_solver;
    message_queue_in_out m_message_queue;
    double m_fixed_dt;
    std::optional<double> m_sleep_timestamp;
    bool m_paused;

    registry_delta_builder m_delta_builder;
    bool m_importing_delta;
    bool m_topology_changed;

    std::vector<entt::entity> m_new_imported_contact_manifolds;

    std::atomic<int> m_reschedule_counter {0};
    std::atomic<bool> m_terminating {false};
    std::atomic<bool> m_terminated {false};
    std::mutex m_terminate_mutex;
    std::condition_variable m_terminate_cv;
};

}

#endif // EDYN_PARALLEL_ISLAND_WORKER_HPP
