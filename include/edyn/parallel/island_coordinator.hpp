#ifndef EDYN_PARALLEL_ISLAND_COORDINATOR_HPP
#define EDYN_PARALLEL_ISLAND_COORDINATOR_HPP

#include <vector>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <entt/fwd.hpp>
#include "edyn/comp/island.hpp"
#include "edyn/math/scalar.hpp"
#include "edyn/util/entity_map.hpp"
#include "edyn/parallel/message.hpp"
#include "edyn/parallel/message_queue.hpp"
#include "edyn/parallel/registry_delta.hpp"

namespace edyn {

class island_worker;
class registry_delta;

class island_coordinator final {

    struct island_info {
        entt::entity m_island_entity;
        entity_set m_entities;
        island_worker *m_worker;
        message_queue_in_out m_message_queue;
        entity_map m_entity_map;
        registry_delta_builder m_delta_builder;
        bool m_pending_flush;

        using registry_delta_func_t = void(entt::entity, const registry_delta &);
        entt::sigh<registry_delta_func_t> m_registry_delta_signal;

        island_info(entt::entity island_entity,
                    island_worker *worker,
                    message_queue_in_out message_queue);
        ~island_info();
        bool empty() const;
        void read_messages();
        void sync();
        void flush();

        template<typename Message, typename... Args>
        void send(Args &&... args) {
            m_message_queue.send<Message>(std::forward<Args>(args)...);
            m_pending_flush = true;
        }
        
        void on_registry_delta(const registry_delta &);

        auto registry_delta_sink() {
            return entt::sink {m_registry_delta_signal};
        }
    };

    void init_new_island_nodes();
    void init_new_non_procedural_island_node(entt::entity);
    entt::entity create_island(double timestamp);
    entt::entity merge_islands(const entity_set &island_entities,
                               const entity_set &new_entities);
    void split_islands();
    void split_island(entt::entity);
    void wake_up_island(entt::entity);
    void refresh_dirty_entities();
    bool should_split_island(const island_topology &);
    void sync();

    void validate();

public:
    island_coordinator(entt::registry &);
    ~island_coordinator();

    void on_construct_island_node(entt::registry &, entt::entity);
    void on_destroy_island_node(entt::registry &, entt::entity);
    void on_construct_island_container(entt::registry &, entt::entity);
    void on_destroy_island_container(entt::registry &, entt::entity);
    void on_registry_delta(entt::entity, const registry_delta &);
    
    void on_construct_constraint(entt::registry &, entt::entity);
    void on_destroy_constraint(entt::registry &, entt::entity);

    void update();

    void set_paused(bool);
    void step_simulation();

    template<typename... Component>
    void refresh(entt::entity entity) {
        static_assert(sizeof...(Component) > 0);
        auto &container = m_registry->get<island_container>(entity);
        for (auto island_entity : container.entities) {
            auto &info = m_island_info_map.at(island_entity);
            info->m_delta_builder.updated<Component...>(entity, *m_registry);
        }
    }

    scalar m_fixed_dt {1.0/60};

private:
    entt::registry *m_registry;
    std::vector<entt::scoped_connection> m_connections;
    std::unordered_map<entt::entity, std::unique_ptr<island_info>> m_island_info_map;

    std::vector<entt::entity> m_new_island_nodes;
    entity_set m_islands_to_split;
    bool m_importing_delta {false};
    bool m_paused {false};
};

}

#endif // EDYN_PARALLEL_ISLAND_COORDINATOR_HPP