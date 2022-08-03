#ifndef EDYN_PARALLEL_ISLAND_WORKER_HPP
#define EDYN_PARALLEL_ISLAND_WORKER_HPP

#include <mutex>
#include <memory>
#include <atomic>
#include <entt/entity/fwd.hpp>
#include <condition_variable>
#include "edyn/collision/raycast.hpp"
#include "edyn/collision/raycast_service.hpp"
#include "edyn/parallel/job.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/dynamics/solver.hpp"
#include "edyn/parallel/message.hpp"
#include "edyn/parallel/entity_graph.hpp"
#include "edyn/parallel/message_dispatcher.hpp"
#include "edyn/parallel/island_manager.hpp"
#include "edyn/util/entity_map.hpp"
#include "edyn/util/polyhedron_shape_initializer.hpp"
#include "edyn/util/registry_operation.hpp"

namespace edyn {

struct settings;
struct extrapolation_result;
class registry_operation_builder;

void island_worker_func(job::data_type &);

/**
 * Simulates one island in a worker thread independently.
 */
class island_worker final {

    enum class state : uint16_t {
        init,
        start,
        step,
        begin_step,
        update_islands,
        broadphase,
        narrowphase,
        solve,
        finish_step,
        raycast
    };

    void init();
    void process_messages();
    bool should_step();
    void begin_step();
    bool run_solver();
    void finish_step();
    void reschedule_now();
    void maybe_reschedule();
    void reschedule_later();
    void do_terminate();
    void maybe_go_to_sleep(entt::entity island_entity);
    void sync();
    void sync_dirty();
    void run_state_machine();
    void update();

    bool all_sleeping();
    void wake_up_affected_islands(const registry_operation_collection &ops);
    void consume_raycast_results();

public:
    island_worker(const settings &settings,
                  const material_mix_table &material_table);

    void reschedule();

    void on_construct_shared_entity(entt::registry &registry, entt::entity entity);
    void on_destroy_shared_entity(entt::registry &registry, entt::entity entity);

    void on_construct_sleeping_tag(entt::registry &registry, entt::entity entity);
    void on_destroy_sleeping_tag(entt::registry &registry, entt::entity entity);

    void on_update_entities(const message<msg::update_entities> &msg);
    void on_set_paused(const message<msg::set_paused> &msg);
    void on_step_simulation(const message<msg::step_simulation> &msg);
    void on_set_settings(const message<msg::set_settings> &msg);
    void on_set_material_table(const message<msg::set_material_table> &msg);
    void on_set_com(const message<msg::set_com> &);
    void on_raycast_request(const message<msg::raycast_request> &);
    void on_apply_network_pools(const message<msg::apply_network_pools> &);
    void on_extrapolation_result(const message<extrapolation_result> &);

    void import_contact_manifolds(const std::vector<contact_manifold> &manifolds);

    bool is_terminated() const;
    bool is_terminating() const;
    void terminate();
    void join();

    friend void island_worker_func(job::data_type &);

private:
    entt::registry m_registry;
    entity_map m_entity_map;
    solver m_solver;
    raycast_service m_raycast_service;
    island_manager m_island_manager;
    polyhedron_shape_initializer m_poly_initializer;

    message_queue_handle<
        msg::set_paused,
        msg::set_settings,
        msg::step_simulation,
        msg::set_com,
        msg::set_material_table,
        msg::update_entities,
        msg::apply_network_pools,
        msg::raycast_request,
        extrapolation_result> m_message_queue;

    double m_last_time;
    double m_step_start_time;

    std::atomic<state> m_state;

    // Set to true when stepping while paused.
    bool m_force_step {false};

    std::unique_ptr<registry_operation_builder> m_op_builder;
    bool m_importing;

    std::atomic<int> m_reschedule_counter {0};

    std::atomic<bool> m_terminating {false};
    std::atomic<bool> m_terminated {false};
    std::mutex m_terminate_mutex;
    std::condition_variable m_terminate_cv;

    job m_this_job;
};

}

#endif // EDYN_PARALLEL_ISLAND_WORKER_HPP
