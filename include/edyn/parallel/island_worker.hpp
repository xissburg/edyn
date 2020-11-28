#ifndef EDYN_PARALLEL_ISLAND_WORKER_HPP
#define EDYN_PARALLEL_ISLAND_WORKER_HPP

#include <mutex>
#include <atomic>
#include <condition_variable>
#include <entt/entt.hpp>
#include "edyn/serialization/memory_archive.hpp"
#include "edyn/parallel/message_queue.hpp"
#include "edyn/parallel/message.hpp"
#include "edyn/parallel/job.hpp"
#include "edyn/parallel/job_dispatcher.hpp"
#include "edyn/time/time.hpp"
#include "edyn/collision/broadphase_worker.hpp"
#include "edyn/collision/narrowphase.hpp"
#include "edyn/dynamics/solver.hpp"
#include "edyn/comp.hpp"
#include "edyn/util/tuple.hpp"
#include "edyn/parallel/registry_delta.hpp"
#include "edyn/collision/contact_manifold.hpp"

namespace edyn {

class job_dispatcher;

void island_worker_func(job::data_type &);

class island_worker final {

    void maybe_split_island();
    void do_terminate();
    void validate_island();
    void init_new_imported_contact_manifolds();

public:
    island_worker(entt::entity island_entity, scalar fixed_dt, message_queue_in_out message_queue);

    virtual ~island_worker() {}

    void on_registry_delta(const registry_delta &delta);

    void sync();

    void update();

    void step();

    void reschedule();

    void on_construct_constraint(entt::registry &, entt::entity);
    void on_destroy_constraint(entt::registry &, entt::entity);

    void on_construct_contact_manifold(entt::registry &, entt::entity);
    
    void on_construct_island_node(entt::registry &, entt::entity);
    void on_update_island_node(entt::registry &, entt::entity);
    void on_destroy_island_node(entt::registry &, entt::entity);

    void on_set_paused(const msg::set_paused &msg);
    void on_step_simulation(const msg::step_simulation &msg);

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
    bool m_paused;

    registry_delta_builder m_delta_builder;
    bool m_importing_delta;
    bool m_splitting_island;
    bool m_topology_changed;

    std::vector<entt::entity> m_new_imported_contact_manifolds;

    std::atomic<bool> m_terminating {false};
    std::atomic<bool> m_terminated {false};
    std::mutex m_terminate_mutex;
    std::condition_variable m_terminate_cv;
};

}

#endif // EDYN_PARALLEL_ISLAND_WORKER_HPP