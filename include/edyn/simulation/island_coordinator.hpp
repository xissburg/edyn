#ifndef EDYN_SIMULATION_ISLAND_COORDINATOR_HPP
#define EDYN_SIMULATION_ISLAND_COORDINATOR_HPP

#include <vector>
#include <memory>
#include <entt/entity/fwd.hpp>
#include "edyn/collision/raycast.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/config/config.h"
#include "edyn/simulation/simulation_worker_context.hpp"
#include "edyn/parallel/message.hpp"
#include "edyn/parallel/message_dispatcher.hpp"
#include "edyn/replication/registry_operation.hpp"
#include "edyn/replication/registry_operation_builder.hpp"

namespace edyn {

class simulation_worker;

/**
 * Manages all simulation islands. Creates and destroys island workers as necessary
 * and synchronizes the workers and the main registry.
 */
class island_coordinator final {

    void init_new_nodes_and_edges();
    void init_new_non_procedural_node(entt::entity);

    void create_worker();
    void insert_to_worker(const std::vector<entt::entity> &nodes,
                          const std::vector<entt::entity> &edges);

    void refresh_dirty_entities();
    void sync();
    void balance_workers();

    struct worker_raycast_context {
        vector3 p0, p1;
        raycast_delegate_type delegate;
        raycast_result result;
    };

public:
    island_coordinator(island_coordinator const&) = delete;
    island_coordinator operator=(island_coordinator const&) = delete;
    island_coordinator(entt::registry &);
    ~island_coordinator();

    void on_construct_graph_node(entt::registry &, entt::entity);
    void on_construct_graph_edge(entt::registry &, entt::entity);

    void on_destroy_graph_node(entt::registry &, entt::entity);
    void on_destroy_graph_edge(entt::registry &, entt::entity);

    void on_step_update(const message<msg::step_update> &);
    void on_raycast_response(const message<msg::raycast_response> &);

    void update();

    void set_paused(bool);
    void step_simulation();

    void set_center_of_mass(entt::entity entity, const vector3 &com);

    // Call when settings have changed in the registry's context. It will
    // propagate changes to island workers.
    void settings_changed();

    void material_table_changed();

    double get_simulation_timestamp() const;

    template<typename Message, typename... Args>
    void send_island_message(entt::entity island_entity, Args &&... args) {
        m_worker_ctx->send<Message>(m_message_queue_handle.identifier, std::forward<Args>(args)...);
    }

    raycast_id_type raycast(vector3 p0, vector3 p1,
                            const raycast_delegate_type &delegate,
                            std::vector<entt::entity> ignore_entities = {}) {
        auto id = m_next_raycast_id++;
        auto &ctx = m_raycast_ctx[id];
        ctx.delegate = delegate;
        ctx.p0 = p0;
        ctx.p1 = p1;

        m_worker_ctx->send<msg::raycast_request>(m_message_queue_handle.identifier, id, p0, p1, ignore_entities);

        return id;
    }

private:
    entt::registry *m_registry;
    std::unique_ptr<simulation_worker_context> m_worker_ctx;
    message_queue_handle<
        msg::step_update,
        msg::raycast_response
    > m_message_queue_handle;

    std::vector<entt::entity> m_new_graph_nodes;
    std::vector<entt::entity> m_new_graph_edges;

    bool m_importing {false};
    double m_timestamp;

    raycast_id_type m_next_raycast_id {};
    std::map<raycast_id_type, worker_raycast_context> m_raycast_ctx;
};

}

#endif // EDYN_SIMULATION_ISLAND_COORDINATOR_HPP
