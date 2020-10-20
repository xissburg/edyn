#ifndef EDYN_PARALLEL_ISLAND_COORDINATOR_HPP
#define EDYN_PARALLEL_ISLAND_COORDINATOR_HPP

#include <vector>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <entt/fwd.hpp>
#include "edyn/parallel/message_queue.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/util/entity_map.hpp"
#include "edyn/math/scalar.hpp"

namespace edyn {

struct constraint;
class island_worker;
class registry_snapshot;

class island_coordinator final {

    struct island_info {
        entt::entity m_entity;
        island_worker *m_worker;
        message_queue_in_out m_message_queue;
        entity_map m_entity_map;

        using registry_snapshot_func_t = void(entt::entity, const registry_snapshot &);
        entt::sigh<registry_snapshot_func_t> m_registry_snapshot_signal;

        island_info(entt::entity entity,
                    island_worker *worker,
                    message_queue_in_out message_queue)
            : m_entity(entity)
            , m_worker(worker)
            , m_message_queue(message_queue)
        {
            m_message_queue.sink<registry_snapshot>().connect<&island_info::on_registry_snapshot>(*this);
        }

        ~island_info() {
            m_message_queue.sink<registry_snapshot>().disconnect(*this);
        }

        auto registry_snapshot_sink() {
            return entt::sink {m_registry_snapshot_signal};
        }

        void on_registry_snapshot(const registry_snapshot &snapshot) {
            m_registry_snapshot_signal.publish(m_entity, snapshot);
        }
    };

    struct destroyed_island_node_info {
        entt::entity entity;
        island_node node;
    };

    void init_new_island_nodes();
    void init_new_non_procedural_island_node(entt::entity);
    entt::entity create_island();
    void process_destroyed_nodes();
    void refresh_dirty_entities();

public:
    island_coordinator(entt::registry &);
    ~island_coordinator();

    void on_construct_island_node(entt::registry &, entt::entity);
    void on_destroy_island_node(entt::registry &, entt::entity);
    void on_destroy_island_container(entt::registry &, entt::entity);
    void on_registry_snapshot(entt::entity, const registry_snapshot &);
    
    void on_construct_constraint(entt::registry &, entt::entity);
    void on_destroy_constraint(entt::registry &, entt::entity);

    void connect_nodes(entt::entity, entt::entity);
    void disconnect_nodes(entt::entity, entt::entity);
    void merge_islands(const std::unordered_set<entt::entity> &);

    void update();

    void set_paused(bool);
    void step_simulation();

    scalar m_fixed_dt {1.0/60};

private:
    entt::registry *m_registry;
    std::vector<entt::scoped_connection> m_connections;
    std::unordered_map<entt::entity, std::unique_ptr<island_info>> m_island_info_map;

    std::vector<entt::entity> m_new_island_nodes;
    std::map<entt::entity, std::vector<entt::entity>> m_destroyed_island_nodes;
    bool m_importing_snapshot {false};
};

}

#endif // EDYN_PARALLEL_ISLAND_COORDINATOR_HPP