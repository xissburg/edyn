#ifndef EDYN_SIMULATION_STEPPER_ASYNC_HPP
#define EDYN_SIMULATION_STEPPER_ASYNC_HPP

#include <vector>
#include <memory>
#include <entt/entity/fwd.hpp>
#include "edyn/collision/query_aabb.hpp"
#include "edyn/collision/raycast.hpp"
#include "edyn/comp/aabb.hpp"
#include "edyn/config/config.h"
#include "edyn/simulation/simulation_worker_context.hpp"
#include "edyn/parallel/message.hpp"
#include "edyn/replication/registry_operation_builder.hpp"
#include "edyn/replication/registry_operation_observer.hpp"

namespace edyn {

/**
 * Manages all simulation islands. Creates and destroys island workers as necessary
 * and synchronizes the workers and the main registry.
 */
class stepper_async final {

    void create_worker();
    void insert_to_worker(const std::vector<entt::entity> &nodes,
                          const std::vector<entt::entity> &edges);

    void sync();

    struct worker_raycast_context {
        vector3 p0, p1;
        raycast_delegate_type delegate;
        raycast_result result;
    };

    struct worker_query_aabb_context {
        AABB aabb;
        query_aabb_delegate_type delegate;
        std::vector<entt::entity> result;
    };

public:
    stepper_async(stepper_async const&) = delete;
    stepper_async operator=(stepper_async const&) = delete;
    stepper_async(entt::registry &);
    ~stepper_async();

    void on_construct_graph_node(entt::registry &, entt::entity);
    void on_construct_graph_edge(entt::registry &, entt::entity);

    void on_destroy_graph_node(entt::registry &, entt::entity);
    void on_destroy_graph_edge(entt::registry &, entt::entity);

    void on_step_update(const message<msg::step_update> &);
    void on_raycast_response(const message<msg::raycast_response> &);
    void on_query_aabb_response(const message<msg::query_aabb_response> &);

    void update();

    void set_paused(bool);
    void step_simulation();

    void set_center_of_mass(entt::entity entity, const vector3 &com);

    // Call when settings have changed in the registry's context. It will
    // propagate changes to island workers.
    void settings_changed();

    void reg_op_ctx_changed();

    void material_table_changed();

    double get_simulation_timestamp() const;

    template<typename Message, typename... Args>
    void send_message_to_worker(Args &&... args) {
        m_worker_ctx->send<Message>(m_message_queue_handle.identifier, std::forward<Args>(args)...);
    }

    raycast_id_type raycast(vector3 p0, vector3 p1,
                            const raycast_delegate_type &delegate,
                            std::vector<entt::entity> ignore_entities = {});

    query_aabb_id_type query_island_aabb(const AABB &aabb,
                                         const query_aabb_delegate_type &delegate);

    query_aabb_id_type query_aabb(const AABB &aabb, const query_aabb_delegate_type &delegate,
                                  bool islands_only = false);

private:
    entt::registry *m_registry;
    std::unique_ptr<simulation_worker_context> m_worker_ctx;
    std::unique_ptr<registry_operation_builder> m_op_builder;
    std::unique_ptr<registry_operation_observer> m_op_observer;
    message_queue_handle<
        msg::step_update,
        msg::raycast_response,
        msg::query_aabb_response
    > m_message_queue_handle;

    bool m_importing {false};

    raycast_id_type m_next_raycast_id {};
    std::map<raycast_id_type, worker_raycast_context> m_raycast_ctx;

    query_aabb_id_type m_next_query_aabb_id {};
    std::map<query_aabb_id_type, worker_query_aabb_context> m_query_aabb_ctx;
};

}

#endif // EDYN_SIMULATION_STEPPER_ASYNC_HPP
