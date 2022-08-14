#ifndef EDYN_NETWORKING_EXTRAPOLATION_JOB_HPP
#define EDYN_NETWORKING_EXTRAPOLATION_JOB_HPP

#include "edyn/networking/extrapolation_input.hpp"
#include "edyn/networking/extrapolation_result.hpp"
#include "edyn/parallel/job.hpp"
#include "edyn/dynamics/solver.hpp"
#include "edyn/replication/entity_map.hpp"
#include "edyn/simulation/island_manager.hpp"
#include "edyn/util/polyhedron_shape_initializer.hpp"
#include <entt/entity/fwd.hpp>
#include <memory>
#include <atomic>

namespace edyn {

class input_state_history;
struct registry_operation_context;
class material_mix_table;
struct settings;

void extrapolation_job_func(job::data_type &);

class extrapolation_job final {

    enum class state {
        init,
        step,
        begin_step,
        update_islands,
        broadphase,
        narrowphase,
        solve,
        finish_step,
        done
    };

    void load_input();
    void init();
    bool should_step();
    void begin_step();
    void finish_step();
    void apply_history();
    void sync_and_finish();
    void run_state_machine();
    void update();

public:
    extrapolation_job(extrapolation_input &&input,
                      const settings &settings,
                      const registry_operation_context &reg_op_ctx,
                      const material_mix_table &material_table,
                      std::shared_ptr<input_state_history> input_history);

    void reschedule();

    bool is_finished() const {
        return m_finished.load(std::memory_order_acquire);
    }

    auto & get_result() {
        return m_result;
    }

    friend void extrapolation_job_func(job::data_type &);

private:
    entt::registry m_registry;
    entity_map m_entity_map;
    solver m_solver;
    island_manager m_island_manager;
    polyhedron_shape_initializer m_poly_initializer;
    extrapolation_input m_input;
    extrapolation_result m_result;

    state m_state;
    double m_start_time;
    double m_current_time;
    unsigned m_step_count {0};
    std::atomic<bool> m_finished {false};

    std::shared_ptr<input_state_history> m_input_history;

    job m_this_job;
};

}

#endif // EDYN_NETWORKING_EXTRAPOLATION_JOB_HPP
