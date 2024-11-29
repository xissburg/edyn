#ifndef EDYN_SIMULATION_SIMULATION_WORKER_HPP
#define EDYN_SIMULATION_SIMULATION_WORKER_HPP

#include <condition_variable>
#include <memory>
#include <atomic>
#include <entt/signal/sigh.hpp>
#include <entt/entity/fwd.hpp>
#include "edyn/collision/raycast.hpp"
#include "edyn/collision/raycast_service.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/dynamics/solver.hpp"
#include "edyn/parallel/message.hpp"
#include "edyn/core/entity_graph.hpp"
#include "edyn/parallel/message_dispatcher.hpp"
#include "edyn/replication/entity_map.hpp"
#include "edyn/replication/registry_operation_builder.hpp"
#include "edyn/replication/registry_operation_observer.hpp"
#include "edyn/simulation/island_manager.hpp"
#include "edyn/util/polyhedron_shape_initializer.hpp"

namespace edyn {

struct settings;
struct extrapolation_result;
class registry_operation_builder;
struct registry_operation_context;

class simulation_worker final {

    void init();
    void deinit();
    void sync();
    void run();
    void update();

    void wake_up_affected_islands(const registry_operation &ops);
    void consume_raycast_results();
    void mark_transforms_replaced();

public:
    simulation_worker(const settings &settings,
                      const registry_operation_context &reg_op_ctx,
                      const material_mix_table &material_table);
    ~simulation_worker();

    void on_construct_shared_entity(entt::registry &registry, entt::entity entity);
    void on_destroy_shared_entity(entt::registry &registry, entt::entity entity);

    void on_update_entities(message<msg::update_entities> &msg);
    void on_set_paused(message<msg::set_paused> &msg);
    void on_step_simulation(message<msg::step_simulation> &msg);
    void on_set_settings(message<msg::set_settings> &msg);
    void on_set_reg_op_ctx(message<msg::set_registry_operation_context> &msg);
    void on_set_material_table(message<msg::set_material_table> &msg);
    void on_set_com(message<msg::set_com> &);
    void on_raycast_request(message<msg::raycast_request> &);
    void on_query_aabb_request(message<msg::query_aabb_request> &);
    void on_query_aabb_of_interest_request(message<msg::query_aabb_of_interest_request> &);
    void on_apply_network_pools(message<msg::apply_network_pools> &);
    void on_extrapolation_result(message<extrapolation_result> &);
    void on_wake_up_residents(message<msg::wake_up_residents> &);
    void on_change_rigidbody_kind(message<msg::change_rigidbody_kind> &);

    void start();
    void stop();

private:
    entt::registry m_registry;
    entity_map m_entity_map;
    raycast_service m_raycast_service;
    island_manager m_island_manager;
    polyhedron_shape_initializer m_poly_initializer;
    solver m_solver;

    message_queue_handle<
        msg::set_paused,
        msg::set_settings,
        msg::set_registry_operation_context,
        msg::step_simulation,
        msg::set_com,
        msg::set_material_table,
        msg::update_entities,
        msg::apply_network_pools,
        msg::wake_up_residents,
        msg::change_rigidbody_kind,
        msg::raycast_request,
        msg::query_aabb_request,
        msg::query_aabb_of_interest_request,
        extrapolation_result> m_message_queue;

    std::unique_ptr<registry_operation_builder> m_op_builder;
    std::unique_ptr<registry_operation_observer> m_op_observer;
    bool m_importing;

    std::atomic<bool> m_running {true};
    std::atomic<bool> m_finished {false};
    std::mutex m_finish_mutex;
    std::condition_variable m_finish_cv;
    double m_accumulated_time {};
    double m_current_time {};
    double m_last_time {};
    double m_sim_time {};
    bool m_paused {false};

    std::vector<entt::scoped_connection> m_connections;
};

}

#endif // EDYN_SIMULATION_SIMULATION_WORKER_HPP
