#ifndef EDYN_PARALLEL_ISLAND_COORDINATOR_HPP
#define EDYN_PARALLEL_ISLAND_COORDINATOR_HPP

#include <vector>
#include <memory>
#include <entt/entity/fwd.hpp>
#include <entt/signal/sigh.hpp>
#include "edyn/collision/raycast.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/config/config.h"
#include "edyn/parallel/island_worker_context.hpp"
#include "edyn/parallel/message.hpp"
#include "edyn/parallel/message_dispatcher.hpp"
#include "edyn/util/registry_operation.hpp"
#include "edyn/util/registry_operation_builder.hpp"

namespace edyn {

class island_worker;

/**
 * Manages all simulation islands. Creates and destroys island workers as necessary
 * and synchronizes the workers and the main registry.
 */
class island_coordinator final {

    void init_new_nodes_and_edges();
    void init_new_non_procedural_node(entt::entity);

    island_worker_index_type create_worker();
    void insert_to_worker(island_worker_index_type worker_index,
                          const std::vector<entt::entity> &nodes,
                          const std::vector<entt::entity> &edges);

    void refresh_dirty_entities();
    void sync();
    void balance_workers();
    void exchange_islands_from_to(island_worker_index_type from_worker,
                                  island_worker_index_type to_worker);

    struct raycast_context {
        unsigned int counter;
        raycast_delegate_type delegate;
        raycast_result result;
    };

public:
    island_coordinator(island_coordinator const&) = delete;
    island_coordinator operator=(island_coordinator const&) = delete;
    island_coordinator(entt::registry &, unsigned short num_island_workers = 0);
    ~island_coordinator();

    void on_construct_graph_node(entt::registry &, entt::entity);
    void on_construct_graph_edge(entt::registry &, entt::entity);

    void on_destroy_graph_node(entt::registry &, entt::entity);
    void on_destroy_graph_edge(entt::registry &, entt::entity);

    void on_destroy_island_worker_resident(entt::registry &, entt::entity);
    void on_destroy_multi_island_worker_resident(entt::registry &, entt::entity);
    void on_destroy_island(entt::registry &, entt::entity);

    void on_step_update(const message<msg::step_update> &);
    void on_entities_received(const message<msg::entities_received_by_worker> &);
    void on_entities_moved(const message<msg::entities_moved> &);

    void on_raycast_response(const message<msg::raycast_response> &);

    void on_destroy_contact_manifold(entt::registry &, entt::entity);

    void update();

    void set_paused(bool);
    void step_simulation();

    void set_center_of_mass(entt::entity entity, const vector3 &com);

    // Call when settings have changed in the registry's context. It will
    // propagate changes to island workers.
    void settings_changed();

    void material_table_changed();

    void batch_nodes(const std::vector<entt::entity> &nodes,
                     const std::vector<entt::entity> &edges);

    double get_worker_timestamp(island_worker_index_type) const;

    void move_non_procedural_into_worker(entt::entity np_entity, island_worker_index_type worker_index);

    void exchange_islands(island_worker_index_type worker_indexA,
                          island_worker_index_type worker_indexB);

    auto contact_started_sink() {
        return entt::sink{m_contact_started_signal};
    }

    auto contact_ended_sink() {
        return entt::sink{m_contact_ended_signal};
    }

    auto contact_point_created_sink() {
        return entt::sink{m_contact_point_created_signal};
    }

    auto contact_point_destroyed_sink() {
        return entt::sink{m_contact_point_destroyed_signal};
    }

    template<typename Message, typename... Args>
    void send_island_message(entt::entity island_entity, Args &&... args) {
        auto &resident = m_registry->get<island_worker_resident>(island_entity);
        auto &ctx = m_worker_ctx[resident.worker_index];
        ctx->send<Message>(m_message_queue_handle.identifier, std::forward<Args>(args)...);
    }

    template<typename It>
    void raycast_workers(It first, It last, vector3 p0, vector3 p1, const raycast_delegate_type &delegate) {
        auto id = m_next_raycast_id++;
        auto &ctx = m_raycast_ctx[id];
        ctx.counter = std::distance(first, last);
        ctx.delegate = delegate;

        for (; first != last; ++first) {
            auto &worker = m_worker_ctx[*first];
            worker->template send<msg::raycast_request>(m_message_queue_handle.identifier, id, p0, p1);
        }
    }

private:
    entt::registry *m_registry;
    std::vector<std::unique_ptr<island_worker_context>> m_worker_ctx;
    message_queue_handle<
        msg::step_update,
        msg::entities_received_by_worker,
        msg::entities_moved,
        msg::raycast_response
    > m_message_queue_handle;

    entt::sigh<void(entt::entity)> m_contact_started_signal;
    entt::sigh<void(entt::entity)> m_contact_ended_signal;
    entt::sigh<void(entt::entity, contact_manifold::contact_id_type)> m_contact_point_created_signal;
    entt::sigh<void(entt::entity, contact_manifold::contact_id_type)> m_contact_point_destroyed_signal;

    std::vector<entt::entity> m_new_graph_nodes;
    std::vector<entt::entity> m_new_graph_edges;

    bool m_importing {false};
    double m_timestamp;

    unsigned int m_next_raycast_id {};
    std::map<unsigned int, raycast_context> m_raycast_ctx;
};

}

#endif // EDYN_PARALLEL_ISLAND_COORDINATOR_HPP
