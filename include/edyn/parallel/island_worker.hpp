#ifndef EDYN_PARALLEL_ISLAND_WORKER_HPP
#define EDYN_PARALLEL_ISLAND_WORKER_HPP

#include <mutex>
#include <memory>
#include <atomic>
#include <optional>
#include <entt/entity/fwd.hpp>
#include <condition_variable>
#include "edyn/parallel/job.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/dynamics/solver.hpp"
#include "edyn/parallel/message.hpp"
#include "edyn/parallel/message_queue.hpp"
#include "edyn/parallel/entity_graph.hpp"
#include "edyn/util/entity_map.hpp"

namespace edyn {

struct settings;
struct extrapolation_result;
class registry_operation_builder;

void island_worker_func(job::data_type &);

/**
 * Simulates one island in a worker thread independently.
 */
class island_worker final {

    enum class state {
        init,
        step,
        begin_step,
        solve,
        broadphase,
        broadphase_async,
        narrowphase,
        narrowphase_async,
        finish_step
    };

    void init();
    void process_messages();
    bool should_step();
    void begin_step();
    bool run_broadphase();
    void finish_broadphase();
    bool run_narrowphase();
    void finish_narrowphase();
    void run_solver();
    void finish_step();
    void reschedule_now();
    void maybe_reschedule();
    void reschedule_later();
    void do_terminate();
    void init_new_shapes();
    void insert_remote_node(entt::entity remote_entity);
    void maybe_go_to_sleep();
    bool could_go_to_sleep();
    void go_to_sleep();
    bool should_split();
    void sync();
    void sync_dirty();
    void update();

public:
    island_worker(entt::entity island_entity, const settings &settings,
                  const material_mix_table &material_table,
                  message_queue_in_out message_queue);

    ~island_worker();

    entt::entity island_entity() const {
        return m_island_entity;
    }

    void reschedule();

    void on_construct_graph_node(entt::registry &, entt::entity);
    void on_destroy_graph_node(entt::registry &, entt::entity);
    void on_destroy_graph_edge(entt::registry &, entt::entity);
    void on_construct_polyhedron_shape(entt::registry &, entt::entity);
    void on_construct_compound_shape(entt::registry &, entt::entity);
    void on_destroy_rotated_mesh_list(entt::registry &, entt::entity);

    void on_island_reg_ops(const msg::island_reg_ops &msg);
    void on_set_paused(const msg::set_paused &msg);
    void on_step_simulation(const msg::step_simulation &msg);
    void on_set_settings(const msg::set_settings &msg);
    void on_set_material_table(const msg::set_material_table &msg);
    void on_wake_up_island(const msg::wake_up_island &);
    void on_set_com(const msg::set_com &);
    void on_apply_network_pools(const msg::apply_network_pools &);
    void on_extrapolation_result(const extrapolation_result &);

    void import_contact_manifolds(const std::vector<contact_manifold> &manifolds);

    entity_graph::connected_components_t split();

    bool is_terminated() const;
    bool is_terminating() const;
    void terminate();
    void join();

    friend void island_worker_func(job::data_type &);

private:
    entt::registry m_registry;
    entt::entity m_island_entity;
    entity_map m_entity_map;
    solver m_solver;
    message_queue_in_out m_message_queue;

    double m_step_start_time;
    std::optional<double> m_sleep_timestamp;

    state m_state;
    std::atomic<bool> m_splitting;

    std::unique_ptr<registry_operation_builder> m_op_builder;
    bool m_importing;
    bool m_destroying_node;
    bool m_topology_changed;
    bool m_pending_split_calculation;
    double m_calculate_split_delay;
    double m_calculate_split_timestamp;

    std::vector<entt::entity> m_new_polyhedron_shapes;
    std::vector<entt::entity> m_new_compound_shapes;

    std::atomic<int> m_reschedule_counter {0};

    std::atomic<bool> m_terminating {false};
    std::atomic<bool> m_terminated {false};
    std::mutex m_terminate_mutex;
    std::condition_variable m_terminate_cv;

    job m_this_job;
};

}

#endif // EDYN_PARALLEL_ISLAND_WORKER_HPP
