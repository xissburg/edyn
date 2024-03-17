#ifndef EDYN_SIMULATION_STEPPER_ASYNC_HPP
#define EDYN_SIMULATION_STEPPER_ASYNC_HPP

#include <vector>
#include <memory>
#include <entt/entity/fwd.hpp>
#include <entt/signal/sigh.hpp>
#include "edyn/collision/query_aabb.hpp"
#include "edyn/collision/raycast.hpp"
#include "edyn/comp/aabb.hpp"
#include "edyn/config/config.h"
#include "edyn/simulation/simulation_worker.hpp"
#include "edyn/parallel/message.hpp"
#include "edyn/replication/registry_operation_builder.hpp"
#include "edyn/replication/registry_operation_observer.hpp"
#include "edyn/util/rigidbody.hpp"

namespace edyn {

/**
 * Steps the simulation asynchronously. It runs as a background job which does
 * the actual simulation and synchronizes it with the main registry.
 */
class stepper_async final {

    void insert_to_worker(const std::vector<entt::entity> &nodes,
                          const std::vector<entt::entity> &edges);

    void sync();

    void calculate_presentation_delay(double current_time, double elapsed, scalar fixed_dt);

    struct worker_raycast_context {
        vector3 p0, p1;
        raycast_delegate_type delegate;
    };

    struct worker_query_aabb_context {
        AABB aabb;
        query_aabb_delegate_type delegate;
    };

public:
    stepper_async(stepper_async const&) = delete;
    stepper_async operator=(stepper_async const&) = delete;
    stepper_async(entt::registry &, double time);

    void on_construct_shared(entt::registry &, entt::entity);

    void on_destroy_graph_node(entt::registry &, entt::entity);
    void on_destroy_graph_edge(entt::registry &, entt::entity);

    void on_step_update(message<msg::step_update> &);
    void on_raycast_response(message<msg::raycast_response> &);
    void on_query_aabb_response(message<msg::query_aabb_response> &);

    void update(double current_time);

    void set_paused(bool);
    void step_simulation();

    void set_center_of_mass(entt::entity entity, const vector3 &com);
    void wake_up_entity(entt::entity entity);
    void set_rigidbody_kind(entt::entity entity, rigidbody_kind kind);

    // Call when settings have changed in the registry's context. It will
    // propagate changes to island workers.
    void settings_changed();

    void reg_op_ctx_changed();

    void material_table_changed();

    double get_simulation_timestamp() const { return m_sim_time; }
    double get_presentation_delay() const { return m_presentation_delay; }

    template<typename Message, typename... Args>
    void send_message_to_worker(Args &&... args) {
        message_dispatcher::global().send<Message>({"worker"},
                                                   m_message_queue_handle.identifier,
                                                   std::forward<Args>(args)...);
    }

    raycast_id_type raycast(vector3 p0, vector3 p1,
                            const raycast_delegate_type &delegate,
                            std::vector<entt::entity> ignore_entities = {});

    query_aabb_id_type query_aabb(const AABB &aabb, const query_aabb_delegate_type &delegate,
                                  bool query_procedural,
                                  bool query_non_procedural,
                                  bool query_islands);

    query_aabb_id_type query_aabb_of_interest(const AABB &aabb, const query_aabb_delegate_type &delegate);

private:
    entt::registry *m_registry;

    simulation_worker m_worker;
    entity_map m_entity_map;

    std::unique_ptr<registry_operation_builder> m_op_builder;
    std::unique_ptr<registry_operation_observer> m_op_observer;
    message_queue_handle<
        msg::step_update,
        msg::raycast_response,
        msg::query_aabb_response
    > m_message_queue_handle;

    bool m_importing {false};
    double m_last_time {};
    double m_sim_time {};
    double m_presentation_delay {};
    std::array<double, 48> m_time_diff_samples {};
    bool m_should_calculate_presentation_delay {false};
    bool m_adjusting_presentation_delay {true};
    bool m_paused {false};

    raycast_id_type m_next_raycast_id {};
    std::map<raycast_id_type, worker_raycast_context> m_raycast_ctx;

    query_aabb_id_type m_next_query_aabb_id {};
    std::map<query_aabb_id_type, worker_query_aabb_context> m_query_aabb_ctx;

    std::vector<entt::scoped_connection> m_connections;
};

}

#endif // EDYN_SIMULATION_STEPPER_ASYNC_HPP
