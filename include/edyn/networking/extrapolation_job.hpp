#ifndef EDYN_NETWORKING_EXTRAPOLATION_JOB_HPP
#define EDYN_NETWORKING_EXTRAPOLATION_JOB_HPP

#include "edyn/networking/extrapolation_input.hpp"
#include "edyn/networking/extrapolation_result.hpp"
#include "edyn/parallel/job.hpp"
#include "edyn/dynamics/solver.hpp"
#include "edyn/util/entity_map.hpp"
#include <entt/entity/fwd.hpp>
#include <memory>
#include <atomic>

namespace edyn {

class comp_state_history;
class material_mix_table;
struct settings;

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

    void load_input();
    void init();
    bool should_step();
    void begin_step();
    void run_solver();
    bool run_broadphase();
    void finish_broadphase();
    bool run_narrowphase();
    void finish_narrowphase();
    void finish_step();
    void create_rotated_meshes();
    void apply_history();
    void sync_and_finish();
    void update();

public:
    extrapolation_job(extrapolation_input &&input,
                      const settings &settings,
                      const material_mix_table &material_table,
                      std::shared_ptr<comp_state_history> state_history);

    void reschedule();

    bool is_finished() const {
        return m_finished.load(std::memory_order_acquire);
    }

    auto & get_result() {
        return m_result;
    }

    void on_destroy_graph_node(entt::registry &, entt::entity);
    void on_destroy_graph_edge(entt::registry &, entt::entity);
    void on_destroy_rotated_mesh_list(entt::registry &, entt::entity);

    friend void extrapolation_job_func(job::data_type &);

private:
    entt::registry m_registry;
    entity_map m_entity_map;
    solver m_solver;
    extrapolation_input m_input;
    extrapolation_result m_result;

    state m_state;
    double m_start_time;
    double m_current_time;
    unsigned m_step_count {0};
    std::atomic<bool> m_finished {false};
    bool m_destroying_node {false};

    std::shared_ptr<comp_state_history> m_state_history;

    job m_this_job;
};

}

#endif // EDYN_NETWORKING_EXTRAPOLATION_JOB_HPP
