#ifndef EDYN_NETWORKING_EXTRAPOLATION_WORKER_HPP
#define EDYN_NETWORKING_EXTRAPOLATION_WORKER_HPP

#include <memory>
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <entt/entity/fwd.hpp>
#include "edyn/networking/extrapolation/extrapolation_modified_comp.hpp"
#include "edyn/networking/extrapolation/extrapolation_request.hpp"
#include "edyn/networking/extrapolation/extrapolation_result.hpp"
#include "edyn/dynamics/solver.hpp"
#include "edyn/replication/entity_map.hpp"
#include "edyn/replication/registry_operation.hpp"
#include "edyn/simulation/island_manager.hpp"
#include "edyn/util/polyhedron_shape_initializer.hpp"
#include "edyn/parallel/message.hpp"
#include "edyn/parallel/message_dispatcher.hpp"

namespace edyn {

class input_state_history;
struct registry_operation_context;
class material_mix_table;
struct settings;

class extrapolation_worker final {

    void init();
    void deinit();
    void init_extrapolation();
    bool should_step();
    void begin_step();
    void finish_step();
    void apply_history();
    void finish_extrapolation();
    void run();
    void extrapolate();

public:
    extrapolation_worker(const settings &settings,
                         const registry_operation_context &reg_op_ctx,
                         const material_mix_table &material_table,
                         std::shared_ptr<input_state_history> input_history,
                         make_extrapolation_modified_comp_func_t *make_extrapolation_modified_comp);

    ~extrapolation_worker();

    void start();
    void stop();

    void set_settings(const edyn::settings &settings);
    void set_material_table(const material_mix_table &material_table);
    void set_registry_operation_context(const registry_operation_context &reg_op_ctx);
    void set_context_settings(std::shared_ptr<input_state_history> input_history,
                              make_extrapolation_modified_comp_func_t *make_extrapolation_modified_comp);

    void on_extrapolation_request(message<extrapolation_request> &msg);
    void on_registry_operation(message<registry_operation> &msg);
    void on_set_settings(message<msg::set_settings> &msg);
    void on_set_reg_op_ctx(message<msg::set_registry_operation_context> &msg);
    void on_set_material_table(message<msg::set_material_table> &msg);
    void on_set_extrapolator_context_settings(message<msg::set_extrapolator_context_settings> &msg);
    void on_push_message();

private:
    entt::registry m_registry;
    entity_map m_entity_map;
    island_manager m_island_manager;
    polyhedron_shape_initializer m_poly_initializer;
    solver m_solver;

    std::unique_ptr<extrapolation_modified_comp> m_modified_comp;

    message_queue_handle<
        extrapolation_request,
        registry_operation,
        msg::set_settings,
        msg::set_registry_operation_context,
        msg::set_material_table,
        msg::set_extrapolator_context_settings> m_message_queue;

    std::unique_ptr<std::thread> m_thread;
    std::atomic<bool> m_running {false};
    std::atomic<bool> m_has_messages {false};
    bool m_has_work {false};

    extrapolation_request m_request;
    entt::sparse_set m_current_entities;

    double m_init_time;
    double m_current_time;
    unsigned m_step_count {0};
    bool m_terminated_early {false};

    std::shared_ptr<input_state_history> m_input_history;

    std::mutex m_mutex;
    std::condition_variable m_cv;
};

}

#endif // EDYN_NETWORKING_EXTRAPOLATION_WORKER_HPP
