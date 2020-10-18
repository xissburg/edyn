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
#include "edyn/collision/broadphase.hpp"
#include "edyn/collision/narrowphase.hpp"
#include "edyn/dynamics/solver.hpp"
#include "edyn/comp.hpp"
#include "edyn/util/tuple.hpp"
#include "edyn/parallel/registry_snapshot.hpp"
#include "edyn/collision/contact_manifold.hpp"

namespace edyn {

class job_dispatcher;

void island_worker_func(job::data_type &);

class island_worker final {

    void refresh_dirty_entities();
    void do_terminate();

public:
    island_worker(entt::entity island_entity, scalar fixed_dt, message_queue_in_out message_queue);

    virtual ~island_worker() {}

    void on_registry_snapshot(const registry_snapshot &snapshot);

    void sync();

    void update();

    void step();

    void reschedule();

    template<typename Comp>
    void on_destroy_component(entt::registry &registry, entt::entity entity) {
        if (m_importing_snapshot) return;
        m_snapshot_builder.template destroyed<Comp>(entity);
    }

    template<typename Comp>
    void on_replace_component(entt::registry &registry, entt::entity entity) {
        if (m_importing_snapshot) return;
        auto &comp = registry.get<Comp>(entity);
        m_snapshot_builder.template updated<Comp>(entity, comp);
    }

    void on_construct_constraint(entt::registry &, entt::entity);
    void on_destroy_constraint(entt::registry &, entt::entity);

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
    broadphase m_bphase;
    narrowphase m_nphase;
    solver m_solver;
    message_queue_in_out m_message_queue;
    double m_fixed_dt;
    bool m_paused;
    registry_snapshot_builder m_snapshot_builder;
    bool m_importing_snapshot;
    std::atomic<bool> m_terminating {false};
    std::atomic<bool> m_terminated {false};
    std::mutex m_terminate_mutex;
    std::condition_variable m_terminate_cv;
};

}

#endif // EDYN_PARALLEL_ISLAND_WORKER_HPP