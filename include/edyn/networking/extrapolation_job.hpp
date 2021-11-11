#ifndef EDYN_NETWORKING_EXTRAPOLATION_JOB_HPP
#define EDYN_NETWORKING_EXTRAPOLATION_JOB_HPP

#include "edyn/parallel/job.hpp"
#include "edyn/dynamics/solver.hpp"
#include "edyn/parallel/message.hpp"
#include "edyn/collision/narrowphase.hpp"
#include "edyn/collision/broadphase_worker.hpp"
#include "edyn/parallel/message_queue.hpp"
#include "edyn/util/entity_map.hpp"
#include "edyn/parallel/island_delta.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

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
    void init_new_imported_contact_manifolds();
    void init_new_shapes();
    void insert_remote_node(entt::entity remote_entity);
    void sync();
    void sync_dirty();
    void apply_history();
    void update();

public:
    extrapolation_job(double target_time, const settings &settings,
                  const material_mix_table &material_table,
                  message_queue_in_out message_queue);

    entt::entity island_entity() const {
        return m_island_entity;
    }

    void on_island_delta(const island_delta &delta);

    void reschedule();

    void on_destroy_contact_manifold(entt::registry &, entt::entity);
    void on_destroy_contact_point(entt::registry &, entt::entity);
    void on_destroy_graph_node(entt::registry &, entt::entity);
    void on_destroy_graph_edge(entt::registry &, entt::entity);
    void on_construct_polyhedron_shape(entt::registry &, entt::entity);
    void on_construct_compound_shape(entt::registry &, entt::entity);
    void on_destroy_rotated_mesh_list(entt::registry &, entt::entity);

    friend void extrapolation_job_func(job::data_type &);

private:
    entt::registry m_registry;
    entt::entity m_island_entity;
    entity_map m_entity_map;
    broadphase_worker m_bphase;
    narrowphase m_nphase;
    solver m_solver;
    message_queue_in_out m_message_queue;

    state m_state;

    std::unique_ptr<island_delta_builder> m_delta_builder;
    bool m_destroying_node;

    double m_target_time;

    std::vector<entt::entity> m_new_imported_contact_manifolds;
    std::vector<entt::entity> m_new_polyhedron_shapes;
    std::vector<entt::entity> m_new_compound_shapes;

    std::vector<island_delta> m_history;

    job m_this_job;
};

}

#endif // EDYN_NETWORKING_EXTRAPOLATION_JOB_HPP
