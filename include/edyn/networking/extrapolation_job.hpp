#ifndef EDYN_NETWORKING_EXTRAPOLATION_JOB_HPP
#define EDYN_NETWORKING_EXTRAPOLATION_JOB_HPP

#include "edyn/collision/contact_manifold.hpp"
#include "edyn/parallel/job.hpp"
#include "edyn/dynamics/solver.hpp"
#include "edyn/parallel/message.hpp"
#include "edyn/collision/narrowphase.hpp"
#include "edyn/collision/broadphase_worker.hpp"
#include "edyn/parallel/message_queue.hpp"
#include "edyn/util/entity_map.hpp"
#include "edyn/parallel/island_delta.hpp"
#include "edyn/networking/packet/transient_snapshot.hpp"
#include "edyn/networking/context/client_networking_context.hpp"
#include "edyn/networking/util/client_pool_snapshot_importer.hpp"
#include <entt/entity/registry.hpp>
#include <atomic>

namespace edyn {

struct extrapolation_completed {
    island_delta delta;
    double timestamp;
};

class non_proc_comp_state_history;

void extrapolation_job_func(job::data_type &);

class extrapolation_job final {

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
    void run_solver();
    bool run_broadphase();
    void finish_broadphase();
    bool run_narrowphase();
    void finish_narrowphase();
    void finish_step();
    void init_new_shapes();
    void insert_remote_node(entt::entity remote_entity);
    void import_manifolds();
    void apply_history();
    void sync_and_finish();
    void update();

public:
    extrapolation_job(double start_time, const settings &settings,
                      const material_mix_table &material_table,
                      std::shared_ptr<client_pool_snapshot_importer> pool_snapshot_importer,
                      non_proc_comp_state_history &state_history,
                      message_queue_in_out message_queue);

    void on_island_delta(const island_delta &delta);
    void on_transient_snapshot(const packet::transient_snapshot &snapshot);

    void reschedule();

    bool is_finished() const {
        return m_finished.load(std::memory_order_relaxed);
    }

    void on_destroy_graph_node(entt::registry &, entt::entity);
    void on_destroy_graph_edge(entt::registry &, entt::entity);
    void on_construct_polyhedron_shape(entt::registry &, entt::entity);
    void on_construct_compound_shape(entt::registry &, entt::entity);
    void on_destroy_rotated_mesh_list(entt::registry &, entt::entity);

    friend void extrapolation_job_func(job::data_type &);

private:
    entt::registry m_registry;
    entity_map m_entity_map;
    solver m_solver;
    message_queue_in_out m_message_queue;
    std::shared_ptr<client_pool_snapshot_importer> m_pool_snapshot_importer;

    state m_state;
    double m_current_time;
    std::atomic<bool> m_finished {false};

    std::unique_ptr<island_delta_builder> m_delta_builder;
    bool m_destroying_node;

    std::vector<entt::entity> m_new_polyhedron_shapes;
    std::vector<entt::entity> m_new_compound_shapes;
    std::vector<contact_manifold> m_imported_manifolds;

    non_proc_comp_state_history *m_state_history;

    job m_this_job;
};

}

#endif // EDYN_NETWORKING_EXTRAPOLATION_JOB_HPP
