#ifndef EDYN_PARALLEL_ISLAND_COORDINATOR_HPP
#define EDYN_PARALLEL_ISLAND_COORDINATOR_HPP

#include <entt/fwd.hpp>
#include "edyn/parallel/message_queue.hpp"
#include "edyn/parallel/island_worker.hpp"

namespace edyn {

struct constraint;

class island_coordinator final {

    struct island_info {
        island_worker *m_worker;
        message_queue_in_out m_message_queue;
        entity_map m_entity_map;

        island_info(island_worker *worker,
                    message_queue_in_out message_queue)
            : m_worker(worker)
            , m_message_queue(message_queue)
        {}
    };

    struct destroyed_island_node_info {
        entt::entity entity;
        island_node node;
    };

    void init_new_island_nodes();
    void destroy_pending_island_nodes();
    void init_new_non_procedural_island_node(entt::entity);
    entt::entity create_island();

public:
    island_coordinator(entt::registry &);
    ~island_coordinator();

    void on_construct_island_node(entt::entity, entt::registry &, island_node &);
    void on_destroy_island_node(entt::entity, entt::registry &);
    void on_destroy_island_container(entt::entity, entt::registry &);
    void on_registry_snapshot(entt::entity, const registry_snapshot &);

    void connect_nodes(entt::entity, entt::entity);
    void disconnect_nodes(entt::entity, entt::entity);
    void merge_islands(entt::entity, entt::entity);

    void update();

    void set_paused(bool);
    void step_simulation();

private:
    entt::registry *m_registry;
    std::vector<entt::scoped_connection> m_connections;
    std::unordered_map<entt::entity, island_info> m_island_info_map;

    std::vector<entt::entity> m_new_island_nodes;
    std::vector<destroyed_island_node_info> m_destroyed_island_nodes;
    bool m_importing_snapshot {false};
};

}

#endif // EDYN_PARALLEL_ISLAND_COORDINATOR_HPP