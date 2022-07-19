#ifndef EDYN_PARALLEL_ISLAND_WORKER_HPP
#define EDYN_PARALLEL_ISLAND_WORKER_HPP

#include <mutex>
#include <memory>
#include <atomic>
#include <entt/entity/fwd.hpp>
#include <condition_variable>
#include "edyn/parallel/job.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/dynamics/solver.hpp"
#include "edyn/parallel/message.hpp"
#include "edyn/parallel/entity_graph.hpp"
#include "edyn/parallel/message_dispatcher.hpp"
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
    void init_new_nodes_and_edges();
    void init_new_shapes();
    void insert_remote_node(entt::entity remote_entity);
    void maybe_go_to_sleep(entt::entity island_entity);
    bool could_go_to_sleep(entt::entity island_entity) const;
    void put_to_sleep(entt::entity island_entity);
    void sync();
    void sync_dirty();
    void update();
    entt::entity create_island();
    void insert_to_island(entt::entity island_entity,
                          const std::vector<entt::entity> &nodes,
                          const std::vector<entt::entity> &edges);
    void merge_islands(const std::vector<entt::entity> &island_entities,
                       const std::vector<entt::entity> &new_nodes,
                       const std::vector<entt::entity> &new_edges);
    void split_islands();
    void wake_up_island(entt::entity island_entity);
    bool all_sleeping();

public:
    island_worker(const std::string &name, const settings &settings,
                  const material_mix_table &material_table, message_queue_identifier coordinator_queue_id);

    ~island_worker();

    void reschedule();

    void on_construct_graph_node(entt::registry &, entt::entity);
    void on_construct_graph_edge(entt::registry &, entt::entity);
    void on_destroy_graph_node(entt::registry &, entt::entity);
    void on_destroy_graph_edge(entt::registry &, entt::entity);
    void on_destroy_island_resident(entt::registry &, entt::entity);
    void on_construct_polyhedron_shape(entt::registry &, entt::entity);
    void on_construct_compound_shape(entt::registry &, entt::entity);
    void on_destroy_rotated_mesh_list(entt::registry &, entt::entity);

    void on_update_entities(const message<msg::update_entities> &msg);
    void on_set_paused(const message<msg::set_paused> &msg);
    void on_step_simulation(const message<msg::step_simulation> &msg);
    void on_set_settings(const message<msg::set_settings> &msg);
    void on_set_material_table(const message<msg::set_material_table> &msg);
    void on_set_com(const message<msg::set_com> &);
    void on_apply_network_pools(const message<msg::apply_network_pools> &);
    void on_extrapolation_result(const extrapolation_result &);

    auto message_queue_id() const {
        return m_message_queue.identifier;
    }

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
    message_queue_handle<
        msg::set_paused,
        msg::set_settings,
        msg::step_simulation,
        msg::set_com,
        msg::set_material_table,
        msg::update_entities,
        msg::apply_network_pools> m_message_queue;
    message_queue_identifier m_coordinator_queue_id;

    double m_last_time;
    double m_step_start_time;

    state m_state;

    std::unique_ptr<registry_operation_builder> m_op_builder;
    bool m_importing;
    bool m_destroying_node;

    std::vector<entt::entity> m_new_graph_nodes;
    std::vector<entt::entity> m_new_graph_edges;
    std::vector<entt::entity> m_new_polyhedron_shapes;
    std::vector<entt::entity> m_new_compound_shapes;
    entt::sparse_set m_islands_to_split;

    std::atomic<int> m_reschedule_counter {0};

    std::atomic<bool> m_terminating {false};
    std::atomic<bool> m_terminated {false};
    std::mutex m_terminate_mutex;
    std::condition_variable m_terminate_cv;

    job m_this_job;
};

}

#endif // EDYN_PARALLEL_ISLAND_WORKER_HPP
